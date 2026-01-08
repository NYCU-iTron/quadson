import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom

def add_collision_to_urdf(tree):
    root = tree.getroot()

    for link in root.findall("link"):
        visual = link.find("visual")
        if visual is not None:
            # Copy origin and geometry from visual
            origin = visual.find("origin")
            geometry = visual.find("geometry")

            if geometry is not None:
                mesh = geometry.find("mesh")
                if mesh is not None:
                    # Create a new collision element
                    collision = ET.Element("collision")
                    collision.set("name", link.get("name") + "_collision")

                    # Copy origin
                    if origin is not None:
                        collision.append(origin)

                    # Copy geometry (mesh)
                    new_geometry = ET.Element("geometry")
                    new_mesh = ET.Element("mesh")
                    new_mesh.set("filename", mesh.get("filename"))
                    new_mesh.set("scale", mesh.get("scale", "1.00000 1.00000 1.00000"))
                    new_geometry.append(new_mesh)

                    collision.append(new_geometry)

                    # Append collision object to the link
                    link.append(collision)
    return tree

def hide_dummy(tree):
    root = tree.getroot()

    # Find all link elements with "dummy" in their name
    for link in root.findall("link"):
        link_name = link.get("name", "")
        if "dummy" in link_name.lower():  # Case-insensitive check for "dummy"
            # Remove all child elements
            for child in list(link):
                link.remove(child)
            
            # Add minimal inertial element
            inertial = ET.Element("inertial")
            mass = ET.SubElement(inertial, "mass")
            mass.set("value", "0.0001")  # Very small mass
            inertia = ET.SubElement(inertial, "inertia")
            inertia.set("ixx", "0.0001")
            inertia.set("ixy", "0")
            inertia.set("ixz", "0")
            inertia.set("iyy", "0.0001")
            inertia.set("iyz", "0")
            inertia.set("izz", "0.0001")
            link.append(inertial)

    return tree

# Usage
input_urdf = "../assets/whole_body/urdf/quadson.urdf"
output_urdf = "../assets/whole_body/urdf/quadson_modified.urdf"

tree = ET.parse(input_urdf)
tree = add_collision_to_urdf(tree)
# tree = hide_dummy(tree)

# Convert tree to a properly formatted string
root = tree.getroot()
rough_string = ET.tostring(root, encoding="utf-8")
reparsed = minidom.parseString(rough_string)
pretty_xml = reparsed.toprettyxml(indent="    ")

# Remove empty lines
pretty_xml = '\n'.join([line for line in pretty_xml.split('\n') if line.strip()])

# Save the formatted URDF
with open(output_urdf, "w", encoding="utf-8") as f:
    f.write(pretty_xml)

print(f"Modified URDF saved to {output_urdf}")