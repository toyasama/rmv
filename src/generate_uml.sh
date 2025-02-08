#!/bin/bash

DIRECTORIES=(
    "rmv_chore/rmv_chore/"
    "rmv_chore/markers_management/"
    "rmv_chore/parameters/"
    "rmv_chore/tf_management/"
    "rmv_chore/topic_management/"
    "rmv_chore/utils/"
    "rmv_chore/visualization/"
)

pyreverse -o puml  --all-ancestors --filter-mode ALL --colorized --no-standalone "${DIRECTORIES[@]}"

if [ $? -eq 0 ]; then
    echo "Diagramme UML généré avec succès !"
else
    echo " Erreur lors de la génération du diagramme UML."
fi
