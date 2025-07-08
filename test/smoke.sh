echo "--- TMC5041 0 ---"
python tmc5041-spi.py 0 0 
echo "--- TMC5041 1 ---"
python tmc5041-spi.py 0 1
echo "--- Air Pressure ---"
python mcp3002-spi.py 0 2
echo "--- Arc Volt ---"
python mcp3002-spi.py 0 3
./gpio.sh

