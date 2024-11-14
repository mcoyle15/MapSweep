from graphviz import Digraph

# Initialize the Digraph
fault_tree = Digraph("Fault Tree: Motor Not Turning", node_attr={'shape': 'box'}, format='png')

# Define top-level event
fault_tree.node("Motor Not Turning", "Motor Not Turning")

# Power Supply Failure branch
fault_tree.node("Power Supply Failure", "Power Supply Failure", shape="ellipse")
fault_tree.edge("Motor Not Turning", "Power Supply Failure", label="OR")

fault_tree.node("Battery Disconnected", "Battery Disconnected", shape="box")
fault_tree.node("Faulty Wiring", "Faulty Wiring", shape="box")
fault_tree.edge("Power Supply Failure", "Battery Disconnected", label="AND")
fault_tree.edge("Power Supply Failure", "Faulty Wiring", label="AND")

# Motor Controller Malfunction branch
fault_tree.node("Motor Controller Malfunction", "Motor Controller Malfunction", shape="ellipse")
fault_tree.edge("Motor Not Turning", "Motor Controller Malfunction", label="OR")

fault_tree.node("Circuit Board Damaged", "Circuit Board Damaged", shape="box")
fault_tree.node("Overheating Issue", "Overheating Issue", shape="box")
fault_tree.edge("Motor Controller Malfunction", "Circuit Board Damaged", label="AND")
fault_tree.edge("Motor Controller Malfunction", "Overheating Issue", label="AND")

# Software Failure branch
fault_tree.node("Software Failure", "Software Failure", shape="ellipse")
fault_tree.edge("Motor Not Turning", "Software Failure", label="OR")

fault_tree.node("Faulty Code Execution", "Faulty Code Execution", shape="box")
fault_tree.node("Communication Error", "Communication Error", shape="box")
fault_tree.edge("Software Failure", "Faulty Code Execution", label="OR")
fault_tree.edge("Software Failure", "Communication Error", label="OR")

# Mechanical Obstruction branch
fault_tree.node("Mechanical Obstruction", "Mechanical Obstruction", shape="ellipse")
fault_tree.edge("Motor Not Turning", "Mechanical Obstruction", label="OR")

fault_tree.node("Object Blocking Shaft", "Object Blocking Shaft", shape="box")
fault_tree.node("Shaft Jammed", "Shaft Jammed", shape="box")
fault_tree.edge("Mechanical Obstruction", "Object Blocking Shaft", label="OR")
fault_tree.edge("Mechanical Obstruction", "Shaft Jammed", label="OR")

# Overload Protection Triggered branch
fault_tree.node("Overload Protection Triggered", "Overload Protection Triggered", shape="ellipse")
fault_tree.edge("Motor Not Turning", "Overload Protection Triggered", label="OR")

fault_tree.node("Exceeding Load Limit", "Exceeding Load Limit", shape="box")
fault_tree.node("Protective Circuit Activated", "Protective Circuit Activated", shape="box")
fault_tree.edge("Overload Protection Triggered", "Exceeding Load Limit", label="AND")
fault_tree.edge("Overload Protection Triggered", "Protective Circuit Activated", label="AND")

# Render the fault tree as an image
fault_tree.render("/mnt/data/motor_not_turning_fault_tree")
