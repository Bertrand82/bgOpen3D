#!/bin/bash

set -e

# Couleurs pour l'affichage
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$PROJECT_DIR/build"
DATA_DIR="$PROJECT_DIR/data"
TEST_DIR="$PROJECT_DIR/tests"
OUTPUT_FILE="$TEST_DIR/merged.ply"
SKIP_BUILD=false

for arg in "$@"; do
    case "$arg" in
        --no-build)
            SKIP_BUILD=true
            ;;
        -h|--help)
            echo "Usage: $0 [--no-build]"
            echo "  --no-build  Exécute le test sans recompiler"
            exit 0
            ;;
        *)
            echo -e "${RED}Option inconnue: $arg${NC}"
            echo "Utilisez --help pour afficher l'aide."
            exit 1
            ;;
    esac
done

echo -e "${YELLOW}=== Test bgOpen3D ===${NC}"

# Créer le répertoire tests s'il n'existe pas
mkdir -p "$TEST_DIR"

# Vérifier que les fichiers de test existent
if [ ! -f "$DATA_DIR/fused_0.ply" ] || [ ! -f "$DATA_DIR/fused_1.ply" ]; then
    echo -e "${RED}Erreur: fichiers de test manquants dans $DATA_DIR${NC}"
    exit 1
fi

# Compiler sauf si --no-build est demandé
if [ "$SKIP_BUILD" = false ]; then
    echo -e "${YELLOW}Construction du projet...${NC}"
    cmake -S "$PROJECT_DIR" -B "$BUILD_DIR"
    cmake --build "$BUILD_DIR" -j"$(nproc)"
else
    echo -e "${YELLOW}Compilation ignorée (--no-build).${NC}"
fi

# Exécuter le test
echo -e "${YELLOW}Exécution du test...${NC}"
"$BUILD_DIR/bgOpen3D" --merge "$OUTPUT_FILE" "$DATA_DIR/fused_0.ply" "$DATA_DIR/fused_1.ply"

# Vérifier le résultat
if [ -f "$OUTPUT_FILE" ]; then
    FILE_SIZE=$(stat -f%z "$OUTPUT_FILE" 2>/dev/null || stat -c%s "$OUTPUT_FILE" 2>/dev/null)
    echo -e "${GREEN}✓ Test réussi!${NC}"
    echo -e "${GREEN}Fichier de sortie: $OUTPUT_FILE ($(numfmt --to=iec-i --suffix=B $FILE_SIZE 2>/dev/null || echo $FILE_SIZE bytes))${NC}"
    exit 0
else
    echo -e "${RED}✗ Échec: fichier de sortie non créé${NC}"
    exit 1
fi
