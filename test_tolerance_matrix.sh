#!/bin/bash

# BRL-CAD Tolerance Integration Test Matrix
# This script runs the test_region_growing program across various tolerance settings

set -e

echo "=========================================="
echo "BRL-CAD Tolerance Integration Test Matrix"
echo "=========================================="

# Ensure test program exists
if [ ! -f "./test_region_growing" ]; then
    echo "Building test_region_growing program..."
    g++ -std=c++17 -I. test_region_growing.cpp -o test_region_growing
fi

# Test matrix configurations
declare -a ABS_TOLERANCES=("0.01" "0.1" "1.0" "5.0")
declare -a REL_TOLERANCES=("0.001" "0.01" "0.05" "0.1")
declare -a VOLUME_DELTAS=("1.0" "5.0" "10.0" "50.0")

echo
echo "Running tolerance test matrix..."
echo "Note: Warnings about volume delta exceeding tolerance are expected and validate the functionality"
echo

TEST_COUNT=0
TOTAL_TESTS=0

# Calculate total tests
for abs_tol in "${ABS_TOLERANCES[@]}"; do
    for rel_tol in "${REL_TOLERANCES[@]}"; do
        for vol_delta in "${VOLUME_DELTAS[@]}"; do
            ((TOTAL_TESTS++))
        done
    done
done

echo "Total test combinations: $TOTAL_TESTS"
echo

# Run test matrix
for abs_tol in "${ABS_TOLERANCES[@]}"; do
    for rel_tol in "${REL_TOLERANCES[@]}"; do
        for vol_delta in "${VOLUME_DELTAS[@]}"; do
            ((TEST_COUNT++))
            echo "Test $TEST_COUNT/$TOTAL_TESTS: abs=$abs_tol, rel=$rel_tol, vol_delta=$vol_delta"
            
            # Run test and capture key metrics
            OUTPUT=$(./test_region_growing \
                --abs-tolerance "$abs_tol" \
                --rel-tolerance "$rel_tol" \
                --volume-delta "$vol_delta" \
                2>&1 | grep -E "Vertices:|Triangles:|WARNING:")
            
            VERTICES=$(echo "$OUTPUT" | grep "Vertices:" | awk '{print $2}')
            TRIANGLES=$(echo "$OUTPUT" | grep "Triangles:" | awk '{print $2}')
            WARNING=$(echo "$OUTPUT" | grep "WARNING:" || echo "No warning")
            
            printf "  Result: %s vertices, %s triangles\n" "$VERTICES" "$TRIANGLES"
            if [[ "$WARNING" != "No warning" ]]; then
                printf "  Volume warning: %s\n" "$WARNING"
            fi
            echo
        done
    done
done

echo "=========================================="
echo "Tolerance test matrix completed successfully!"
echo "All $TOTAL_TESTS test combinations executed."
echo "=========================================="

# Run edge case tests
echo
echo "Running edge case tolerance tests..."

echo "1. Extreme strict tolerances:"
./test_region_growing --abs-tolerance 0.001 --rel-tolerance 0.0001 --volume-delta 0.1 | grep -E "Vertices:|Triangles:|WARNING:" || true

echo
echo "2. Very relaxed tolerances:"
./test_region_growing --abs-tolerance 100.0 --rel-tolerance 1.0 --volume-delta 500.0 | grep -E "Vertices:|Triangles:|WARNING:" || true

echo
echo "Edge case tests completed!"
echo "=========================================="