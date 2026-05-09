# Plan de développement (pas-à-pas) + tests associés — Localisation caméra (COLMAP + DB + 2D→3D + PnP)

Ce document décrit un plan itératif pour implémenter un pipeline C++ qui :
- lit un modèle COLMAP (`sparse/0/{cameras,images,points3D}.{bin|txt}`) + `database.db`
- extrait des features sur `imageCurrent` (format compatible COLMAP)
- sélectionne des images candidates
- charge keypoints/descriptors candidates depuis la DB
- matche `imageCurrent` ↔ candidates (2D↔2D)
- convertit les matches en correspondances **2D(current) → 3D(modèle)**
- estime la pose via **OpenCV `solvePnPRansac`**
- produit : `(R,t)`, centre caméra `C=-R^T t`, quaternion, yaw/pitch/roll.

## Hypothèses / contraintes
- Descriptors **COLMAP style** : `uint8`, matrice `Nx128` (`cv::Mat CV_8U`)
- Keypoints **COLMAP style** : `float`, matrice `Nx4` (`x,y,scale,orientation`) en `CV_32F`
- Une seule caméra dans le modèle COLMAP (un `camera_id`)
- `imageCurrent` est fourni comme **fichier** (chemin)
- Dépendances typiques : **OpenCV**, **Eigen**, **SQLite3**, framework de tests (**GoogleTest** ou **Catch2**)

---

## Étape 0 — Squelette projet + build + exécution tests (smoke)
### Développement
1. Créer la structure :
   - `include/loc/...`
   - `src/...`
   - `tests/...`
   - `tests/assets/...` (datasets et fichiers de test)
2. CMake :
   - une lib `loc` (ton code)
   - un exécutable `loc_cli` (outil manuel)
   - un exécutable `loc_tests` (tests unitaires/intégration)
3. (Optionnel) `clang-format` + preset CI.

### Tests associés
- **Build test** : compilation locale (et CI si tu ajoutes GitHub Actions).
- **Smoke test** : `loc_tests` s’exécute et retourne code 0.

Critère de succès : le projet compile et les tests “vides” tournent.

---

## Étape 1 — Chargement du modèle COLMAP (sparse/0)
### Développement
Implémenter `ColmapModelLoader::LoadFromSparseDir()` :
- Support `.bin` et `.txt` (selon `prefer_binary`)
- Charger :
  - `cameras`
  - `images` + `points2D` (avec `point3D_id`)
  - `points3D` (id → xyz)
- Vérifications importantes :
  - `point3D_id == -1` => pas de correspondance 3D
  - cohérence basique des tailles (`points2D` non vide si l’image a des keypoints)

### Tests associés
#### 1.1 Unit test “parsing minimal (txt)”
- Mettre un mini modèle dans `tests/assets/sparse_minimal_txt/` (fichiers `.txt`).
- Vérifier :
  - 1 caméra chargée
  - N images chargées
  - certains `point3D_id == -1`
  - `points3D[known_id].xyz` = valeur attendue

#### 1.2 Test “bin vs txt” (si tu supportes les deux)
- Sur un dataset de test contenant les deux formats :
  - charger `.bin`
  - charger `.txt`
  - vérifier : mêmes cardinalités + IDs, et `xyz` proches (tolérance float)

Critère de succès : le loader reconstruit correctement les structures.

---

## Étape 2 — Intrinsics : construction de `K` + `dist`
### Développement
Implémenter `BuildIntrinsicsFromColmap(model_name, w, h, params)` :
- Construire `K` :
  - `K = [[fx,0,cx],[0,fy,cy],[0,0,1]]`
- Construire `dist` en fonction des modèles supportés (minimum : celui de ton dataset)
- Gérer “unknown model” proprement (exception/status).

### Tests associés
#### 2.1 Unit test PINHOLE
- Input : `PINHOLE`, params `[fx, fy, cx, cy]`
- Assert :
  - `K(0,0)=fx`, `K(1,1)=fy`, `K(0,2)=cx`, `K(1,2)=cy`, `K(2,2)=1`
  - dist vide (ou taille 0)

#### 2.2 Unit test OPENCV (si utilisé)
- Input : params `[fx, fy, cx, cy, k1, k2, p1, p2]` (selon COLMAP)
- Assert : dist contient bien les coefficients attendus

Critère de succès : `K` et `dist` corrects pour les modèles nécessaires.

---

## Étape 3 — Accès `database.db` COLMAP : keypoints + descriptors
### Développement
Implémenter `ColmapDatabase` (SQLite, read-only) :
- `LoadKeypoints(image_id)` :
  - query `keypoints` (BLOB)
  - décoder en floats
  - produire `cv::Mat` **Nx4** `CV_32F`
- `LoadDescriptors(image_id)` :
  - query `descriptors` (BLOB)
  - décoder en `uint8`
  - produire `cv::Mat` **Nx128** `CV_8U`
- Vérifier tailles BLOB → dimensions (N, 4) et (N, 128).

### Tests associés
#### 3.1 Unit test “DB fake minimal”
- Générer ou committer `tests/assets/db_minimal.db`
- Insérer une entrée `image_id=1` avec :
  - 2 keypoints (2x4 floats)
  - 2 descriptors (2x128 bytes)
- Assert :
  - types (`CV_32F` / `CV_8U`)
  - dimensions correctes
  - valeurs exactes (sur quelques champs)

#### 3.2 Test cohérence reconstruction ↔ DB
- Charger un modèle + la DB associée.
- Pour une image candidate donnée :
  - vérifier (si vrai sur ton dataset) : `N_db_keypoints == model.images[image_id].points2D.size()`
- Si pas garanti : transformer en “assert soft” (warning) ou utiliser un dataset maîtrisé.

Critère de succès : lecture DB robuste et formats corrects.

---

## Étape 4 — Extraction features sur `imageCurrent` en format COLMAP
### Développement
Implémenter `ColmapStyleSiftExtractor::Extract(gray)` :
- Sorties :
  - `KeypointsF32`: `Nx4` float (x,y,scale,orientation)
  - `DescriptorsU8`: `Nx128` uint8
- Objectif de cette étape : **format + stabilité**, pas forcément “identique bit-à-bit à COLMAP”.

### Tests associés
#### 4.1 Unit test “shape + types”
- Charger une image `tests/assets/img.png`
- Extract
- Assert :
  - `keypoints.cols==4`, `keypoints.type()==CV_32F`
  - `descriptors.cols==128`, `descriptors.type()==CV_8U`
  - `rows(keypoints)==rows(descriptors)`

#### 4.2 Test de non-régression (plage)
- Sur une image fixe, vérifier `N_features` dans une plage plausible
  - exemple : `500 <= N <= 5000` (selon paramètres)
- But : détecter une régression majeure.

Critère de succès : extraction stable, sorties bien typées.

---

## Étape 5 — Matching 2D↔2D (current ↔ candidate)
### Développement
Implémenter `DescriptorMatcher::Match(desc_current, desc_candidate)` :
- KNN (k=2) + ratio test (Lowe)
- option cross-check (désactivée par défaut)
- gérer les cas vides (pas de descripteurs).

### Tests associés
#### 5.1 Unit test “matching identity”
- Construire des descripteurs synthétiques où `A == B`
- Assert : matches 1-1 attendus (au moins sur un sous-ensemble)

#### 5.2 Unit test “ratio test”
- Construire un cas où le 2ᵉ voisin est trop proche → match rejeté
- Assert : filtrage ok

Critère de succès : matching déterministe et filtrage correct.

---

## Étape 6 — Conversion matches 2D↔2D → correspondances 2D(current) ↔ 3D(modèle)
### Développement
Implémenter `CorrespondenceBuilder::Build2D3D(...)` :
- Pour chaque match `(idx_current, idx_candidate)` :
  1. `point3D_id = candidate_image.points2D[idx_candidate].point3D_id`
  2. si `point3D_id == -1` → ignorer
  3. sinon `XYZ = model.points3D[point3D_id].xyz`
  4. `pt2d = (keypoints_current[idx_current].x, y)`
  5. ajouter correspondance `(pt2d, XYZ)`
- Accumuler sur plusieurs candidates.

### Tests associés
#### 6.1 Unit test “ignore -1”
- Candidate avec `point3D_id` = `{-1, 10, -1}`
- Matches vers ces indices
- Assert : seule la correspondance avec `point3D_id=10` est gardée

#### 6.2 Unit test “xyz mapping”
- `points3D[10] = (1,2,3)`
- Assert : `pt3d_w == (1,2,3)`

Critère de succès : correspondances 2D→3D correctes et filtrées.

---

## Étape 7 — Estimation de pose : PnP + RANSAC + sorties dérivées
### Développement
Implémenter `PnpEstimator::EstimatePoseW2C(corr, intr)` :
- Convertir vers `object_points` (3D) et `image_points` (2D)
- Appeler `cv::solvePnPRansac(...)`
- `Rodrigues(rvec) -> R`
- `t = tvec`
- `C = -R^T t`
- `DerivePoseOutputs` :
  - quaternion à partir de `R`
  - yaw/pitch/roll : **fixer une convention explicite** (ex: ZYX : yaw(Z), pitch(Y), roll(X))

### Tests associés
#### 7.1 Test synthétique “pose connue” (très important)
- Générer une pose ground-truth `(R_gt, t_gt)`
- Générer des points 3D devant la caméra
- Projeter avec `K` (sans dist au début)
- Ajouter bruit pixel faible
- Lancer PnP RANSAC
- Assert :
  - erreur rotation < ~1°
  - erreur translation/centre < seuil (selon échelle)
  - inliers >= un pourcentage attendu

#### 7.2 Test robustesse aux outliers
- Ajouter 30–50% de fausses correspondances
- Assert :
  - `success == true`
  - `num_inliers >= minInliers`

Critère de succès : PnP robuste, sorties cohérentes.

---

## Étape 8 — Sélection d’images candidates
### Développement
D’abord `HardcodedCandidateSelector` :
- exemple : prendre les `max_candidates` plus petits `image_id` (triés)
Ensuite (optionnel) :
- sélection par proximité à la pose initiale (distance entre centres cam du modèle et `C0_world`)
- nécessite conversion des poses COLMAP (attention conventions cam->world vs world->cam)

### Tests associés
#### 8.1 Unit test “hardcoded deterministic”
- modèle avec image_ids `{5,2,9}`
- `max_candidates=2`
- Assert : retourne `{2,5}` (selon la règle choisie)

Critère de succès : sélection reproductible.

---

## Étape 9 — Pipeline complet (intégration)
### Développement
Implémenter `LocalizationPipeline::Run(inputs)` :
1. Charger reconstruction
2. Récupérer **la seule caméra** et construire intrinsics
3. Charger `imageCurrent` (`cv::imread`) + conversion grayscale
4. Extraire features current
5. Sélectionner candidates
6. Pour chaque candidate :
   - charger keypoints/descriptors DB
   - matcher
   - construire correspondances 2D→3D
7. Concaténer toutes les correspondances
8. Lancer PnP RANSAC
9. Retourner outputs (intrinsics, candidates, compteurs, pose)

### Tests associés
#### 9.1 Test intégration dataset réel minimal
Dans `tests/assets/dataset1/` :
- `sparse/0/*`
- `database.db`
- `query.jpg` (idéalement une image du set)
Asserts :
- `success == true`
- `num_inliers >= minInliers`
- `total_correspondences_2d3d > 0`

#### 9.2 Test “aucune correspondance”
- `query` sans overlap (image random)
- Assert :
  - `success == false` (ou `num_inliers < minInliers`)
  - message/status explicite (si tu implémentes une gestion d’erreurs)

Critère de succès : le pipeline fonctionne de bout en bout et échoue proprement si nécessaire.

---

## Étape 10 — CLI + tests end-to-end de régression
### Développement
Créer `loc_cli` :
- arguments :
  - `--sparse <.../sparse/0>`
  - `--db <.../database.db>`
  - `--image <imageCurrent>`
  - `--initC x y z`
  - `--ypr yaw pitch roll`
- affiche :
  - `R` et `t` (monde → caméra)
  - `C = -R^T t`
  - quaternion
  - yaw/pitch/roll

### Tests associés
#### 10.1 “Golden-ish output” (avec tolérances)
- Sur dataset figé, vérifier :
  - succès
  - inliers dans une plage
  - (optionnel) composantes de `C` dans une plage
- Attention : solveurs peuvent varier → privilégier des tolérances.

---

# Recommandations anti-pièges
- **Convention yaw/pitch/roll** : la fixer au début, la documenter, et écrire un test unitaire dédié.
- Gérer les erreurs avec un statut explicite :
  - DB n’a pas `image_id`
  - taille keypoints DB ≠ `points2D` reconstruction
  - pas assez de correspondances
  - solvePnPRansac échoue
- Valider PnP d’abord **sans distorsion**, puis ajouter `dist`.
- Sur des jeux de test réels, garder des tests “non stricts” (plages) pour éviter les flakes.

---