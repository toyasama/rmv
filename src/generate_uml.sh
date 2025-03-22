#!/bin/bash

DIRECTORIES=(
    "rmv_chore/"
    "rmv_library/markers_management/"
    "rmv_library/parameters/"
    "rmv_library/tf_management/"
    "rmv_library/topic_management/"
    "rmv_library/utils/"
)

pyreverse -o puml  --filter-mode ALL --colorized --no-standalone "${DIRECTORIES[@]}"

if [ $? -eq 0 ]; then
    echo "Diagramme UML généré avec succès !"
else
    echo " Erreur lors de la génération du diagramme UML."
fi
