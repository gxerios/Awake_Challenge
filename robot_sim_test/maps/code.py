import numpy as np
from scipy.ndimage import gaussian_filter
from imageio import imread
import trimesh

def numpy2stl_simplified(data, output_stl_path, extrusion_height=0.2, mask_val=240):
    # Limiter l'extrusion aux zones non blanches
    data = np.where(data < mask_val, extrusion_height, 0)  # Extrude only non-white areas to a fixed height
    vertices = []
    faces = []
    height, width = data.shape

    # Construction de la géométrie avec une simplification pour alléger le fichier STL
    for y in range(0, height - 1, 2):  # Sauter des lignes pour alléger la complexité
        for x in range(0, width - 1, 2):  # Sauter des colonnes
            if data[y, x] > 0:  # Si la zone est à extruder
                # Quatre coins avec hauteur d'extrusion fixe
                z = data[y, x]
                v0 = [x, y, 0]
                v1 = [x + 2, y, 0]
                v2 = [x + 2, y + 2, 0]
                v3 = [x, y + 2, 0]
                v4 = [x, y, z]
                v5 = [x + 2, y, z]
                v6 = [x + 2, y + 2, z]
                v7 = [x, y + 2, z]
                
                # Ajouter les sommets
                base_idx = len(vertices)
                vertices.extend([v0, v1, v2, v3, v4, v5, v6, v7])
                
                # Ajouter les faces pour chaque mur et le couvercle supérieur
                faces.extend([
                    [base_idx, base_idx + 1, base_idx + 5], [base_idx, base_idx + 5, base_idx + 4],  # Face avant
                    [base_idx + 1, base_idx + 2, base_idx + 6], [base_idx + 1, base_idx + 6, base_idx + 5],  # Face droite
                    [base_idx + 2, base_idx + 3, base_idx + 7], [base_idx + 2, base_idx + 7, base_idx + 6],  # Face arrière
                    [base_idx, base_idx + 3, base_idx + 7], [base_idx, base_idx + 7, base_idx + 4],  # Face gauche
                    [base_idx + 4, base_idx + 5, base_idx + 6], [base_idx + 4, base_idx + 6, base_idx + 7]   # Couvercle supérieur
                ])

    vertices = np.array(vertices)
    faces = np.array(faces)
    
    # Créer et exporter le maillage
    mesh_data = trimesh.Trimesh(vertices=vertices, faces=faces)
    mesh_data.export(output_stl_path)
    print(f"STL file created: {output_stl_path}")

def png_to_3d_stl_simplified(image_path, output_stl_path, extrusion_height=0.2, background_threshold=240):
    img = imread(image_path)
    
    if img.ndim == 3 and img.shape[2] == 4:  # Image en RGBA
        img = img[:, :, 0] + img[:, :, 3]  # Combine red and alpha channels
    elif img.ndim == 3:  # Image en RGB
        img = img[:, :, 0]  # Utiliser seulement le canal rouge
    elif img.ndim == 2:  # Image en niveaux de gris
        pass  # Pas besoin de transformation
    
    img = gaussian_filter(img, sigma=2)  # Lissage de l’image
    numpy2stl_simplified(img, output_stl_path, extrusion_height=extrusion_height, mask_val=background_threshold)

# Exemple d’utilisation
png_to_3d_stl_simplified("map3.png", "output_model.stl", extrusion_height=0.2)
