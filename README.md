# Annotation Tool
<p>
    <a href="#"><img alt="Python" src="https://img.shields.io/badge/Python-14354C.svg?logo=python&logoColor=white"></a>
    <a href="#"><img alt="PyQT5" src="https://img.shields.io/badge/PyQT5-49D.svg?logo=Qt&logoColor=white"></a>
    <a href="#"><img alt="Markdown" src="https://img.shields.io/badge/Markdown-000000.svg?logo=markdown&logoColor=white"></a>
    <a href="#"><img alt="Visual Studio Code" src="https://img.shields.io/badge/Visual%20Studio%20Code-ad78f7.svg?logo=visual-studio-code&logoColor=white"></a>
    <a href="#"><img alt="Linux" src="https://img.shields.io/badge/Linux-0078D6?logo=linux&logoColor=white"></a>
    <a href="#"><img alt="Windows" src="https://img.shields.io/badge/Windows-0078D6?logo=windows&logoColor=white"></a>
</p>


<h1 id="Dependecies">➤ Dependecies</h1>

Note: It is recommended to create a virtual environment under Anaconda for installation.


1) **Python 3.8+** 

2) **Install Python libraries requirements:**

    The `requirements.txt` file should list all Python libraries that your notebooks
    depend on, and they will be installed using:
    ```bash
    $ pip install -r requirements.txt
    ```

3) **Build Resource File:**

    This command utilizes the PyQt5 Resource Compiler (pyrcc5) tool. Its purpose is to compile a resource file (.qrc file) into Python code, allowing you to utilize these resources in your application, such as images, fonts, style sheets, etc.
    ```bash
    $ pyrcc5 -o modules/resources/resources.py modules/resources.qrc
    ```

<h1 id="Usage">➤ Usage</h1>

Note: 
<font color="yellow">Please check the labels in the 'default_classes.txt' file first</font>. You don't need to specify the following CLI arguments.

***Tracking Labeling Tool*** :

- Run :

    ```bash
    # methods 1: setting in the UI
    $ python trackingTool.py
    ```
    
    * [<font color="yellow">Check Label Table</font>]: Check the label tags of classes.
    
    * [<font color="yellow">Video Files List Path</font>]: Drag and drop video files into the field or use the Add Btn to add your own paths.
    
    * [<font color="yellow">Save Path</font>]: Root path for saving, where a folder will be automatically created for each video.
    <p>
        <img src="./demo/trackingSelectUI.png" height=350px width=400px>
    </p>

    ```bash
    # methods 2: CLI
    $ python trackingTool.py -i <path-to-video-dir> -c <path-to-txt-classes> -o <path-to-ouput-folder>
    ```

    Description of CLI all arguments:
    - `--video_dir` : Path to the input video directory.

    - `--class_file` : Path to the file containing class names.

    - `--save_folder` : Folder to save the results.

- Keyboard operation:

    | ID         | Describe            |
    |------------|---------------------|
    | Tab/Enter  |Enter Label Painter. |
    | Esc        |Quit.                | 


    Label Painter

    | ID       | Describe             |
    |----------|----------------------|
    | w        |Create Box.           |
    | Delete   |Select Box and Delete.|
    | Ctrl+C   |Copy the previous box.|
    | Esc      |Exit Painter Mode.    |

***Manual Labeling Tool*** :

```bash
# methods 1: setting in the UI
$ python labelingTool.py
```

* [<font color="yellow">Open Directory</font>] : corresponds to the 'images' folder

* [<font color="yellow">Change Saving Directory</font>] : corresponds to the 'labels' folder.
<p>
    <img src="./demo/labelingUI.png" height=350px width=700px>
</p>

```bash
# method 2: custom yourself
$ python labelingTool.py -i <path-to-image-dir> -c <path-to-txt-classes> -o <path-to-label-folder>
```

Description of CLI all arguments:

- `--image_dir` : Path to the directory containing images.

- `--class_file` : Path to the file containing class names.

- `--save_dir` : Path to the directory to save labels.

