#!/bin/bash
# encoding: utf-8

ISA=$1
echo "Collecting artifacts for $ISA"

TARGET_FOLDER="../ece_562_benchmarks/$ISA/benchmarks"

cp ./blocked-matmul "$TARGET_FOLDER"
cp ./queens "$TARGET_FOLDER"
cp ./sha "$TARGET_FOLDER"

# Issues compiling
# cp ./BFS "$TARGET_FOLDER"
