# Transformation

## Steps to Run

Clone the project
```bash
  git clone https://github.com/Pratham-Pandey/Transformation.git
```

Go to the project directory
```bash
  cd Transformation
```

### Files setup
* All the lidar pointcluod ".npy" files are in binary and we need to resave them into ".npy" properly.
* For this open "convert_to_npy.py" and change the value of the following variables:
  * **file_dir**: Change it to the path of the directory where the original ".npy" files are.
  * **save_dir**: Change it to the path of the directory where you want to save the output files.
* Once the changes are made, execute the "convert_to_npy.py" file.
  
### Solving Problems: 1,2,3
* There is a single file "task.py" containing solution to all there problems.
* Open "task.py" and change the value of the following variables:
  * **lidar_data_dir**: Change this to same path as the variable **save_dir** from the previous step.
  * **camera_data_dir**: Change it to the path of the directory containing the images of the ChArUco board(.png files).
* Once the changes are made, execute the "convert_to_npy.py" file.

## Interpreting the Output
