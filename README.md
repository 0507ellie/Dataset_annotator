# Annotation Tool (v1.0.1)
<p>
    <a href="#"><img alt="Python" src="https://img.shields.io/badge/Python-14354C.svg?logo=python&logoColor=white"></a>
    <a href="#"><img alt="PyQT5" src="https://img.shields.io/badge/PyQT5-49D.svg?logo=Qt&logoColor=white"></a>
    <a href="#"><img alt="Markdown" src="https://img.shields.io/badge/Markdown-000000.svg?logo=markdown&logoColor=white"></a>
    <a href="#"><img alt="Visual Studio Code" src="https://img.shields.io/badge/Visual%20Studio%20Code-ad78f7.svg?logo=visual-studio-code&logoColor=white"></a>
    <a href="#"><img alt="Linux" src="https://img.shields.io/badge/Linux-0078D6?logo=linux&logoColor=white"></a>
    <a href="#"><img alt="Windows" src="https://img.shields.io/badge/Windows-0078D6?logo=windows&logoColor=white"></a>
</p>

<p class="center">
    <img src="./demo/labelingPage.png" height=170px width=230px>
    <img src="./demo/trackingPage.png" height=170px width=230px>
    <img src="./demo/formatPage.png" height=170px width=230px>
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
    $ sudo pyrcc5 -o resources/resources.py resources/resources.qrc
    ```

<h1 id="Usage">➤ Usage</h1>

***Tracking Labeling Tool*** :

- Run :

    <p>
        <img src="./demo/trackingUI.png" height=350px width=700px>
    </p>

    ```bash
    # methods 1: setting in the UI
    $ python trackingTool.py

    # methods 2: custom yourself
    $ python trackingTool.py -i <path-to-video-dir> -c <path-to-txt-classes> -o <path-to-ouput-dir>
    ```

    Description of CLI all arguments:
    - `--video_dir` : Path to the input video directory.

    - `--class_file` : Path to the file containing class names.

    - `--save_dir` : Folder to save the results(Default: Under the path of the current video)

- Keyboard operation:

    Tracker View

    <p>
        <img src="./demo/trackingKeyboard.png" height=200px width=550px>
    </p>

    Label Painter

    | ID       | Describe             |
    |----------|----------------------|
    | w        |Create Box.           |
    | Delete   |Select Box and Delete.|
    | Ctrl+C   |Copy the previous box.|
    | Esc      |Exit Painter Mode.    |


***Manual Labeling Tool*** :

- Run :

    <p>
        <img src="./demo/labelingUI.png" height=350px width=700px>
    </p>

    ```bash
    # methods 1: setting in the UI
    $ python labelingTool.py

    # method 2: custom yourself
    $ python labelingTool.py -i <path-to-image-dir> -c <path-to-txt-classes> -o <path-to-label-dir>
    ```

    Description of CLI all arguments:

    - `--image_dir` : Path to the directory containing images.

    - `--class_file` : Path to the file containing class names.

    - `--save_dir` : Path to the directory to save labels.

- Keyboard operation:

    <p>
        <img src="./demo/labelingKeyboard.png" height=350px width=700px>
    </p>