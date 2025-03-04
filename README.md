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
* When we execute the **task.py** file, 2 image will pop up with the following header:
  * **Detected Aruco Markers**: It highlights the detected Aruco Markers.
  * **Detected Checker Corners and Frame**: It highlights the corners of the detected checker. Also it draws a frame on the center of the board representgn its orientation.
 ![img_1](https://github.com/user-attachments/assets/1270bd21-14e4-46e1-ab05-ae2775b145b0)


>  [!NOTE]
> Initially the 2 images would be overlaping each other. We need to drag one image from other.

* Next press any key to continue.
* The next window will display the filtered lidar points in blue and the fitted circle in red.
![img_2](https://github.com/user-attachments/assets/7b9b6fa3-0a36-413c-8983-c621f0815086)

* Press "q" to quit the window.
* The final transformation matrix between the lidar and camera along with other information are printed in the terminal.

* The **output** folder in the repo contains the detected markers.

>  [!IMPORTANT]
> In every path you set, make sure '/' is added at the end of the path.

