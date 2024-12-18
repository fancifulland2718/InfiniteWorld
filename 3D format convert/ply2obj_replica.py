import pymeshlab
import os

print("Loading PLY file...")
ms = pymeshlab.MeshSet()

try:
    ms.load_new_mesh("./apartment_0/mesh.ply")
    print("PLY file loaded successfully.")
except Exception as e:
    print(f"Error loading PLY file: {e}")
    exit()

try:
    ms.save_current_mesh("./apartment_0/output.obj")
    print("OBJ file saved successfully.")
except Exception as e:
    print(f"Error saving OBJ file: {e}")
