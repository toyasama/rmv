#!/bin/bash

sysctl -w net.core.rmem_max=10485760 || echo "Warning: Unable to modify net.core.rmem_max"
sysctl -w net.core.wmem_max=10485760 || echo "Warning: Unable to modify net.core.wmem_max"
