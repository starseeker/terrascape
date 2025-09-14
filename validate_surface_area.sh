#!/bin/bash

# TerraScape Surface Area Validation Script
# This script provides the recommended sanity check to ensure the surface area 
# of the mesh component approximates that of the terrain data.

SCRIPT_DIR="$(dirname "$0")"
DIAGNOSTIC_TOOL="$SCRIPT_DIR/surface_area_check"
DIAGNOSTIC_SOURCE="/tmp/surface_area_check.cpp"

# Function to compile the diagnostic tool if needed
compile_diagnostic_tool() {
    if [ ! -f "$DIAGNOSTIC_TOOL" ]; then
        echo "Compiling surface area diagnostic tool..."
        
        # Create the diagnostic tool source if it doesn't exist
        if [ ! -f "$DIAGNOSTIC_SOURCE" ]; then
            echo "Diagnostic source not found, using existing compiled version..."
            return 0
        fi
        
        g++ -std=c++17 -O2 -o "$DIAGNOSTIC_TOOL" "$DIAGNOSTIC_SOURCE"
        if [ $? -ne 0 ]; then
            echo "Failed to compile diagnostic tool"
            exit 1
        fi
        echo "Diagnostic tool compiled successfully"
    fi
}

# Function to validate mesh against terrain
validate_mesh() {
    local terrain_file="$1"
    local mesh_file="$2"
    
    if [ ! -f "$terrain_file" ]; then
        echo "Error: Terrain file '$terrain_file' not found"
        return 1
    fi
    
    if [ ! -f "$mesh_file" ]; then
        echo "Error: Mesh file '$mesh_file' not found"
        return 1
    fi
    
    echo "Validating mesh '$mesh_file' against terrain '$terrain_file'..."
    "$DIAGNOSTIC_TOOL" "$terrain_file" "$mesh_file"
}

# Main script logic
if [ $# -lt 2 ]; then
    echo "TerraScape Surface Area Validation Script"
    echo "Usage: $0 <terrain.pgm> <mesh.obj>"
    echo ""
    echo "This script provides the recommended sanity check to ensure the surface area"
    echo "of the mesh component approximates that of the terrain data."
    echo ""
    echo "Example:"
    echo "  $0 crater.pgm terrain_mesh.obj"
    exit 1
fi

# Use existing diagnostic tool if available
if [ -f "/tmp/surface_area_check" ]; then
    DIAGNOSTIC_TOOL="/tmp/surface_area_check"
    echo "Using existing diagnostic tool: $DIAGNOSTIC_TOOL"
elif [ ! -f "$DIAGNOSTIC_TOOL" ]; then
    echo "Diagnostic tool not found. Please compile it first or use the existing one."
    exit 1
fi

# Run validation
validate_mesh "$1" "$2"