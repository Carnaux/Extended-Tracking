
# Implementation of AR extended tracking function based on ARToolKit and ORB-SLAM


## 1. Project Introduction

This project is to research the traditional AR engine ARToolKit and the open source visual SLAM solution ORB-SLAM, to achieve the integration of visual SLAM technology on the basis of the existing capabilities of the ARToolKit engine to add extended tracking functions to enhance the AR application. Robustness and user experience.


## 2. Project compile and run

### 2.1 Preparation

Operating environment: Ubuntu16.04

-Compiling environment: gcc, g ++ (C ++ 11), Cmake (above 2.8)

-Related library dependencies: This program is formed by the combination of ARToolKit project and ORB-SLAM2 project, so the library dependency of this program is the union of ARToolKit and ORB-SLAM2 library dependencies. For the addition of the library dependencies of both, please refer to [ARToolKit5-github](https://github.com/artoolkit/artoolkit5) and [ORB-SLAM2-github](https://github.com/raulmur/ORB_SLAM2)

. If you encounter problems, you can also refer to `problems you may encounter during compilation.

### 2.2 Program compilation
From a Linux terminal, use the cd command to go to the Extended-Tracking root directory and enter:
```
chmod + x build.sh
./build.sh
```
You can automatically complete the compilation of the entire extension tracking program; its compilation process is:
1. Go to the `Thirdparty/DBoW2` directory and compile the` DBoW2` library required for the ORB-SLAM2 part.
2. Go to the `Thirdparty/g2o` directory and compile the` g2o` library required for ORB-SLAM2.
3. Go to the `lib/SRC` directory and compile the related libraries required by the ARToolKit section according to the` makefile` file.
4. According to the `CmakeLists.txt` file in the root directory, first compile the shared library` libORB_SLAM2.so` required by the `ORB-SLAM2` section, and then compile the extended trace program example.


After compiling, you will get the `DBoW2` library under `Thirdparty/DBoW2/lib`, the `g2o` library under` Thirdparty/ g2o/lib`, and the ARToolKit related library and ORB-SLAM2 shared library under `lib` `libORB_SLAM2.so`, the executable file` extended-tracking` for the extended tracking program example under `bin`.

### 2.3 Program running
1. Print the "pinball.jpg" picture file in the "Related Reference Materials" directory to A4 paper as the program's natural image template.
2. Open a command line terminal, enter the following code and press Enter (specify the camera driver type called by ARToolKit):

    ```
    export ARTOOLKIT5_VCONF = "-device = LinuxV4L2"
    ```
3. Under the `Extended-Tracking` directory, go to the` bin` directory:

    ```
    cd bin
    
    ```
    
    Run the program:
    ```
    ./extended-tracking Vocabulary / ORBvoc.txt TUM1.yaml
    ```
    The `Vocabulary/ORBvoc.txt` parameter is the path of the ORB dictionary file, and the `TUM1.yaml` parameter holds the camera internal parameters and related setting information required by the ORB-SLAM2 program; these two parameters are mainly used by the ORB-SLAM2 part of the program to initialize
use.

4. If you want the program to call the USB camera, the device number of the connected USB camera should be `video0` (used by ARToolKit)
video0 device as the camera), you can use the following command to view the current camera device:

    ```
    ls/dev/v*
    ```

    The following information will be displayed on the terminal:
    ![](http://static.zybuluo.com/LiTAOo/wn5t0m8ufphty6jor5a5hpuu/QQ%E5%9B%BE%E7%89%8720180706161706.png)

    If it is a laptop computer, the Linux system defaults the built-in front camera as the ` video0 ` device. When the computer is running normally, the USB camera is connected, and the USB camera will become the ` video1 ` device at this time. In order to set the USB camera as ` video0 ` device, you can insert the USB camera and turn it on when the computer is off. The USB camera will be automatically set as the ` video0 ` device when the computer is turned on.


5. The screenshot of the program running effect is as follows:
    ![](https://litaooooo.github.io/page-examples/extended-tracking.png)

### 2.4 Limpeza do programa

No diretório `Extended-Tracking`, digite:

````
chmod + x clean.sh
./clean.sh

````
Você pode limpar todos os arquivos intermediários, arquivos de biblioteca e arquivos executáveis ​​gerados durante o processo de compilação.

### 2.5 Outros problemas relacionados
- ` Como criar e treinar modelos de imagens naturais personalizados? `

    Você pode consultar os arquivos de referência relacionados / ARToolKit para acabamento de unidade / ARToolkit para acabamento de unidade .pdf`. Depois de obter os arquivos `.fset`,` .fset3`, `.iset`, coloque-os em O `Extended-Tracking / bin / DataDFT` substitui o arquivo no formato correspondente e modifica as informações relevantes no arquivo` bin / Data2 / markers.dat`.

- ` Como calibrar os parâmetros internos da câmera atual? `

    Você pode consultar o arquivo de referência relacionado / ARToolKit para unity use finish / ARToolkit para unity use finish.pdf` Depois de obter o arquivo `camera_para.dat`, coloque-o em` bin / Data2` e substitua o arquivo pelo mesmo nome E modifique as informações de referência interna no arquivo `bin / TUM1.yaml` de acordo com os resultados da calibração.

Observe que os aplicativos envolvidos no processo acima podem ser executados no sistema operacional ` Windows ` executando os programas relacionados no diretório `Materiais de referência relacionados / versões do ARToolKit (download do site oficial) / ARToolkit5_windows / ARToolKit5 / bin`.

### 2.6 Demonstração do efeito do programa
Consulte: [Demonstração da função de demonstração de rastreamento de extensão AR](https://v.youku.com/v_show/id_XMzgyMTI5MDE2OA==.html)