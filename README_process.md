# LES ENTRES:

## Modele colmap:
- Fichiers COLMAP
- sparse/0/cameras.bin (ou cameras.txt)
- sparse/0/images.bin (ou images.txt)
- sparse/0/points3D.bin (osolvePnPu points3D.txt)
- database.db correspondant à cette reconstruction (mêmes `image_id`)

## Données d’entrée
- imageCurrent (le fichier imagedont il faut localiser la caméra)
- pose initiale supposée :
  - position : ( C_0 = (x_0, y_0, z_0) )
  - orientation : yaw/pitch/roll

## Donnée de sortie (à générer par le pipe)

- pose estimée (monde → caméra) : ( (R, t) )
- centre caméra (position dans le monde) : [ C = -R^\top t ]
- orientation finale : quaternion et yaw/pitch/roll (dérivée de (R))

---

# Préparer les intrinsics de la caméra (K, dist)
- Depuis cameras.bin , récupère :
  - modèle : PINHOLE, OPENCV, OPENCV_FISHEYE, SIMPLE_RADIAL, etc.
  - paramètres : (f_x, f_y, c_x, c_y) + distorsion
  - Construis :matrice intrinsèque : [ K = \begin{bmatrix} f_x & 0 & c_x \ 0 & f_y & c_y \ 0 & 0 & 1 \end{bmatrix} ]
  - vecteur distorsion dist

----

# DESCRITION DU TRAITEMENT

## 1) Extraire les features de imageCurrent avec colmap
- keypoints_current (positions 2D)
- descriptors_current (vecteurs de description)
- Contrainte :
- extracteur SIFT de COLMAP (cohérence maximale)

## 2) Sélectionner des images candidates dans le modèle (proches de la pose initiale)
- Dans un premier temps, prevoir une methode hard codé.

## 3) Charger keypoints + descriptors des images candidates depuis database.db
- Pour chaque image candidate (via image_id) :

- lire la table keypoints : (BLOB) → tableau des keypoints 2D
- lire la table descriptors : (BLOB) → matrice (N \times D)

## 4) Matcher imageCurrent avec chaque candidate (2D ↔ 2D)
- Pour chaque candidate img_j :

- matcher descriptors_current ↔ descriptors_j

## 5) Convertir les matches 2D↔2D en correspondances 2D(current) ↔ 3D(model)
C’est l’étape déterminante : produire des correspondances 2D→3D.

### 5.1 Récupérer le point3D_id d’un keypoint dans l’image candidate
Pour l’image img_j, à l’indice idx_j :
point3D_id = images[img_j].points2D[idx_j].point3D_id
si point3D_id == -1 : pas de correspondance 3D (on ignore ce match)
sinon : on a un lien vers un point 3D du modèle

### 5.2 Récupérer la coordonnée 3D
- Dans points3D.bin/.txt :   XYZ = points3D[point3D_id].xyz

### 5.3 Construire les paires 2D↔3D
Ajouter une correspondance :

2D : pt2d = keypoints_current[idx_current].xy  
3D : pt3d = points3D[point3D_id].xyz  

En accumulant sur plusieurs images candidates, tu obtiens généralement des dizaines à centaines de correspondances 2D↔3D.

## 6) Estimer la pose (position): PnP + RANSAC (OpenCV)
Avec :

- object_points : ( {X_k} ) en 3D (repère monde géoréférencé)
- image_points : ( {x_k} ) en 2D pixels
- intrinsics : K, distorsion : dist
- Lancer :

solvePnPRansac(object_points, image_points, K, dist, rvec, tvec, ...)
Résultat :

- une pose monde → caméra (selon conventions OpenCV)
- conversion en matrice :
Rodrigues(rvec) -> R
- centre caméra monde : [ C = -R^\top t ]





#Prompt suivant:
  
  - Je voux implementer cela en cpp compilé avec COLMAP en tant que bibliothèque C++ (donc  utiliser colmap::Database, colmap::ReadBinary...) et OpenCv : propose les header .hpp
  - Est ce qu'une telle application serai embarquable sur Jetson nano Nvidia ?
  - Peux tu estimer le temps de calcul pour Relocalisation / pose estimation (mon use-case) sur Jetson nano nvidia ? 
    - Moins de 3 secondes pour extraction + matching 20 images si bien preparé. (Peut aller a moins de 1s)