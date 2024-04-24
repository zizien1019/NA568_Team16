# Android Studio Setup
The Android Studio IDE can be downloaded [here](https://developer.android.com/studio?gad_source=1&gclid=CjwKCAjw26KxBhBDEiwAu6KXt9xJpCalnDTE7JICAHzDQWsQN_PKbyNYdl6o0rNav8LPQDlxV7bteRoCXh4QAvD_BwE&gclsrc=aw.ds). Normally, you can either build the app using Java or Kotlin for the back-end, and using XML for the front-end.

## Project Structure
The essential folders and files are:

    .
    ├── app                                                               
    │   ├── src                                                            # The main directory for the source files.
    │   │   ├── main                                                       # Contains the primary code and resources.
    │   │   │   ├── AndroidManifest.xml                                    # Configures the app's name, icon, and permissions.
    │   │   │   ├── res                                                    # Resources folder containing layouts and other UI elements.
    │   │   │   │   ├── layout                                             # Holds XML front-end files defining the user interface.
    │   │   │   │   │   ├── biner_base.xml                                 # XML layouts for HSV segmentation activity.
    │   │   │   │   │   ├── activity_reference_input.xml                   # XML layouts for taking input from the user.
    │   │   │   │   │   ├── camera_view.xml                                # XML layouts for the main activity.
    │   │   │   ├── java/.../na568Teamproject_MohammedAlanUsmanBahru/      # Java back-end source code directory.
    │   │   │   │   │   │   ├── SeekBarHsvSeg.java                         # Manages the user interface for a feature involving an HSV color model.
    │   │   │   │   │   │   ├── ReferenceInput.java                        # Handles user input for reference settings.
    │   │   │   │   │   │   ├── MainActivity.java                          # The main activity managing the core functionality.
    │   │   │   │   │   │   ├── NoiseEstimator.java                        # Manages noise estimation for sensors and updating noise parameters.
    │   │   │   │   │   │   ├── KalmanFilter.java                          # Implements a Kalman filter for estimating and updating the state.
    └── ...

## Dependencies
For running the app via Android Studio, this application uses the following libraries:
* OpenCV 4.1.0 (can be explored [here](https://opencv.org/releases/page/4/) or directly downloaded [here](https://sourceforge.net/projects/opencvlibrary/files/4.1.0/opencv-4.1.0-android-sdk.zip/download))

## Building and Developing the Project Files
  1) Download or clone the repository.
  2) Open this 'android_app_team16' project in your Android Studio IDE. Check and click "Trust Project".
  3) In the Menu Bar try:\
       **Build** &rarr; **Rebuild Project**.
  4) If the build is successful:
      - In your Android device, turn on the USB debugging:\
            **Settings** &rarr; **About phone** &rarr; tap **Build number** x 7 times &rarr; allow for being developer &rarr; go back to **Settings** &rarr; **Systems** &rarr; **Developer options** &rarr; scroll down &rarr; turn on **USB debugging**.
      - Connect your Android device to your computer via USB cable.
      - In the Android Studio IDE press *Run* (or pressing control+R) button at the top of the window.
   5) If the build is not successful:
      - Download [OpenCV for Android dependency](https://sourceforge.net/projects/opencvlibrary/files/4.1.0/opencv-4.1.0-android-sdk.zip/download)) and extract it.
      - In the Android Studio IDE:
        - Load the dependency\
          **File** &rarr; **New** &rarr; **Import Module** &rarr; **Browse** &rarr; **OpenCV-android-sdk** (the OpenCV dependency you have extracted) &rarr; **sdk**  &rarr; click **Open** button &rarr; change module name e.g.: ***opencv*** &rarr; click **Finish**
        - Link the dependency\
          **File** &rarr; **Project Structure** &rarr; **Dependencies** &rarr; in *Modules* click **app** folder &rarr; click **+** (add dependency) in *Declared Dependencies* &rarr; click **3 Module Dependency** &rarr; checkmark **opencv** &rarr; click **OK**  &rarr; if there is Modules **other** than *app* and *opencv* (e.g.: opencv2), please remove it by clicking **-** (Remove Module) and 'yes' button &rarr; click **OK**.
        - Configuring the app *build.gradle* file\
          at the leftmost side menu make sure that **Project** tab is activated &rarr; in the *dropdown* arrow at the top left IDE window click **Project** &rarr; extend **app** by clicking at the arrow &rarr; click and open **build.gradle** in *app* directory &rarr; **remove** the line of *implementation project(path: ':opencv2')* but **keep** *implementation project(path: ':opencv2')* &rarr; click menu bar **File** &rarr; **Sync Project with Gradle Files**.
        - After sucessful, do the step in the point [4](https://github.com/zizien1019/NA568_Team16/blob/ac16694f9cce5ca201592b6c1486c34a0073577a/android_app_team16/README.md?plain=1#L33-L36).
6) Modify and develop the Android app, commonly by modifying code in [Java](https://github.com/zizien1019/NA568_Team16/tree/main/android_app_team16/app/src/main/java/com/mbsbahru/na568Teamproject_MohammedAlanUsmanBahru) and [XML](https://github.com/zizien1019/NA568_Team16/tree/main/android_app_team16/app/src/main/res/layout) directories.
