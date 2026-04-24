#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import yaml
import os
import sys
import re

class FlowList(list):
    pass

def flow_list_rep(dumper, data):
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)

yaml.add_representer(FlowList, flow_list_rep)

def recursively_convert_to_flow(data):
    """Converts all standard Python lists in the dict into FlowLists"""
    if isinstance(data, dict):
        for k, v in data.items():
            data[k] = recursively_convert_to_flow(v)
    elif isinstance(data, list):
        return FlowList(data)
    return data

def format_yaml_grids(yaml_text):
    """ Post-processes the YAML string to format lists into neat grids and separate root nodes """
    # 1. Format the 3xN Grids (Your existing logic)
    pattern = re.compile(r'^(\s*)(cad_points|ordered_cad_points):\s*\[(.*?)\]', re.MULTILINE | re.DOTALL)
    
    def replacer(match):
        indent = match.group(1)
        key = match.group(2)
        numbers_str = match.group(3)
        
        numbers = [float(x) for x in numbers_str.replace('\n', '').split(',') if x.strip()]
        
        formatted_lines = []
        for i in range(0, len(numbers), 3):
            row = numbers[i:i+3]
            row_str = ", ".join([f"{num: 9.6f}" for num in row])
            formatted_lines.append(f"{indent}   {row_str},")
            
        grid_str = "\n".join(formatted_lines)
        return f"{indent}{key}: [\n{grid_str}\n{indent}]"

    processed_yaml = pattern.sub(replacer, yaml_text)

    # 2. Inject an empty line before any top-level hierarchy key
    # Matches a newline followed by a word and a colon (no leading spaces)
    processed_yaml = re.sub(r'\n([a-zA-Z0-9_-]+:)', r'\n\n\1', processed_yaml)

    return processed_yaml

def get_geometric_signatures(points):
    n = len(points)
    signatures = np.zeros((n, n - 1))
    for i in range(n):
        distances = []
        for j in range(n):
            if i != j:
                distances.append(np.linalg.norm(points[i] - points[j]))
        signatures[i] = np.sort(distances)
    return signatures

def parse_aimtool_file(filepath):
    """ Parses Aimooe .aimtool files dynamically based on marker count. """
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Cannot find aimtool file at: {filepath}")
    
    with open(filepath, 'r') as file:
        # Strip whitespace and ignore empty lines
        lines = [line.strip() for line in file.readlines() if line.strip()]

    if len(lines) < 3:
        raise ValueError("The .aimtool file is empty or corrupted.")

    try:
        # Line index 2 contains the number of markers
        num_markers = int(lines[2])
    except ValueError:
        raise ValueError(f"Expected integer on line 3 for marker count, got: {lines[2]}")

    if len(lines) < 3 + num_markers:
        raise ValueError("The .aimtool file is missing coordinate lines based on the declared marker count.")

    cam_points = []
    
    # Loop exactly 'num_markers' times starting from line 4 (index 3)
    for i in range(3, 3 + num_markers):
        parts = lines[i].split()
        if len(parts) < 3:
            raise ValueError(f"Coordinate line {i+1} is malformed: {lines[i]}")
            
        # Extract X, Y, Z and convert mm to meters
        x = float(parts[0]) / 1000.0
        y = float(parts[1]) / 1000.0
        z = float(parts[2]) / 1000.0
        
        cam_points.append([x, y, z])

    return np.array(cam_points)

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        
        # --- Declare Parameters ---
        self.declare_parameter('yaml_path', '')
        self.declare_parameter('target_node_name', '')
        self.declare_parameter('aimtool_path', '')

        yaml_path = self.get_parameter('yaml_path').get_parameter_value().string_value
        node_name = self.get_parameter('target_node_name').get_parameter_value().string_value
        aimtool_path = self.get_parameter('aimtool_path').get_parameter_value().string_value

        self.get_logger().info(f"--- Starting Calibration for '{node_name}' ---")

        # 1. Load CAD points directly from YAML
        try:
            with open(yaml_path, 'r') as file:
                config_data = yaml.safe_load(file) or {}
                
            params = config_data.get(node_name, {}).get('ros__parameters', {})
            cad_flat = params.get('cad_points', [])
            
            if not cad_flat:
                self.get_logger().error(f"Could not find 'cad_points' under '{node_name}'")
                sys.exit(1)
                
            unordered_cad = np.array(cad_flat).reshape(-1, 3)
            expected_markers = len(unordered_cad)
            
        except Exception as e:
            self.get_logger().error(f"Failed to read YAML: {e}")
            sys.exit(1)

        # 2. Load Camera points from Aimtool file
        try:
            cam_points = parse_aimtool_file(aimtool_path)
        except Exception as e:
            self.get_logger().error(f"Aimtool Parse Error: {str(e)}")
            sys.exit(1)

        if len(cam_points) != expected_markers:
            self.get_logger().error(f"Marker mismatch! CAD has {expected_markers}, Aimtool has {len(cam_points)}")
            sys.exit(1)

        # 3. Calculate Signatures & Match
        cad_signatures = get_geometric_signatures(unordered_cad)
        cam_signatures = get_geometric_signatures(cam_points)

        matched_cad_indices = []
        for cam_sig in cam_signatures:
            best_match_idx = -1
            lowest_diff = float('inf')
            for i, cad_sig in enumerate(cad_signatures):
                diff = np.sum(np.abs(cam_sig - cad_sig))
                if diff < lowest_diff:
                    lowest_diff = diff
                    best_match_idx = i
            matched_cad_indices.append(best_match_idx)

        # Symmetry Check
        if len(set(matched_cad_indices)) != expected_markers:
            self.get_logger().error("Geometric ambiguity detected. Are your CAD markers perfectly symmetrical?")
            sys.exit(1)

        # 4. Reorder and write to YAML
        ordered_cad_points = unordered_cad[matched_cad_indices]
        yaml_array = [float(x) for x in ordered_cad_points.flatten()]

        config_data[node_name]['ros__parameters']['ordered_cad_points'] = yaml_array

        # Convert lists to flow style
        config_data = recursively_convert_to_flow(config_data)

        try:
            # 1. Dump to a string instead of directly to the file
            yaml_str = yaml.dump(config_data, default_flow_style=False, sort_keys=False)
            
            # 2. Run our custom grid formatter
            formatted_yaml_str = format_yaml_grids(yaml_str)

            # 3. Write the beautifully formatted string to the file
            with open(yaml_path, 'w') as file:
                file.write(formatted_yaml_str)
                
            self.get_logger().info(f"Calibration successful! YAML updated.")
        except Exception as e:
            self.get_logger().error(f"Failed to write YAML: {e}")
            sys.exit(1)

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

if __name__ == '__main__':
    main()