#!/bin/sh
echo "Running Spotless pre-commit hook..."
# This script runs spotlessApply before committing.
./gradlew spotlessApply -q
# Add any changes made by spotlessApply to the commit.
git add -u
