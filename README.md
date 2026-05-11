### bgOpen3D

### pour compiler 
```bash
cmake --preset linux-local
cmake --build --preset linux-local
./build/bgOpen3D
```

## Usage

Usage:

- Merge:
	/home/bertrand/workspaceCpp/bgOpen3D/build/bgOpen3D --merge output.ply input1.ply input2.ply [input3.ply ...]
- Poisson:
	/home/bertrand/workspaceCpp/bgOpen3D/build/bgOpen3D --poisson [--depth value] [--width value] [--scale value] [--threads value] [--voxel value] [--auto-depth] [--linear-fit] output.ply input1.ply input2.ply [input3.ply ...]
	Valeurs par defaut:
	- --depth 8
	- --width 0.0
	- --scale 1.1
	- --threads -1
	- --voxel 0.0
	- --auto-depth desactive
	- --linear-fit desactive
	Plages recommandees:
	- --depth: 6 a 9 (6-7 pour limiter la RAM, 8-9 pour plus de detail)
	- --width: laisser 0.0 dans la plupart des cas (auto)
	- --scale: 1.0 a 1.2 (1.1 conseille)
	- --threads: 1 a 8 pour stabilite memoire, -1 pour tous les coeurs
	- --voxel: 0.002 a 0.02 selon l'echelle du nuage (plus grand = moins de RAM)
	- --auto-depth: recommande pour gros nuages ou machines limitees en RAM
	- --linear-fit: optionnel, a activer seulement si le resultat vous convient visuellement
- Ball pivoting:
	/home/bertrand/workspaceCpp/bgOpen3D/build/bgOpen3D --buildMeshBallPivoting [--radius value] output.ply input1.ply input2.ply [input3.ply ...]
- Post-merge cleanup (via bgOpen3D):
	/home/bertrand/workspaceCpp/bgOpen3D/build/bgOpen3D --post-merge [--voxel size] [--dedup-eps eps] [--skip-dedup] input.ply output.ply
	Valeurs par defaut:
	- --voxel: desactive (0.0)
	- --dedup-eps: 0.0 (deduplication exacte)
	- --skip-dedup: desactive (deduplication active)
	Plages possibles:
	- --voxel: >= 0 (0.0 ou omis = desactive; > 0 = downsampling, plus grand = moins de points)
	- --dedup-eps: >= 0 (0.0 = exacte; > 0 = tolerance, ex: 1e-8 a 1e-6, plus grand = plus agressif)
	- --skip-dedup: drapeau booléen (absent = deduplication active; present = deduplication desactivee)
- Post-merge cleanup (standalone):
	/home/bertrand/workspaceCpp/bgOpen3D/build/point_cloud_post_merge [--voxel size] [--dedup-eps eps] [--skip-dedup] input.ply output.ply
	Valeurs par defaut:
	- --voxel: desactive (0.0)
	- --dedup-eps: 0.0 (deduplication exacte)
	- --skip-dedup: desactive (deduplication active)
	Plages possibles:
	- --voxel: >= 0 (0.0 ou omis = desactive; > 0 = downsampling, plus grand = moins de points)
	- --dedup-eps: >= 0 (0.0 = exacte; > 0 = tolerance, ex: 1e-8 a 1e-6, plus grand = plus agressif)
	- --skip-dedup: drapeau booléen (absent = deduplication active; present = deduplication desactivee)
