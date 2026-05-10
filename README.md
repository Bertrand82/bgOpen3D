### bgOpen3D

### pour compiler 
'''bash
mkdir -p build
cd build
cmake ..
cmake --build .
./bgOpen3D
'''

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
