#!/bin/bash

COMMANDS_FILE="commands.txt"

if [ ! -f "$COMMANDS_FILE" ]; then
    echo "Error: File '$COMMANDS_FILE' not found in the current directory."
    exit 1
fi

while IFS= read -r command; do
    [[ -z "$command" ]] && continue

    gnome-terminal -- bash -c "$command; exec bash"
done < "$COMMANDS_FILE"