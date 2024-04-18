# 3dmirror
- clone this repo including submodules
```bash
git clone --recurse-submodules https://github.com/FlorianMehnert/3dmirror.git
```
## current progress
![invalid borders](./images/invalid_borders.png)
![webcam](./images/discarded_triangles_webcam.png)
![sad mirror](./images/sad_mirror.png)
![parralaxed_holodeck](./images/probably_working_occlusion.png)

# cgv usage
- see [instructions](https://github.com/lintianfang/coding_issues_cgv/blob/master/how%20to%20use%20%20cgv_framework.txt) for common issues
## kinect issues
- in Windows make sure AZURE_KINECT_SDK environment variable is set to the [azure kinect sdk directory](https://github.com/microsoft/Azure-Kinect-Sensor-SDK)
- for 1.4.1 C:\Program Files\Azure Kinect SDK v1.4.1\sdk
- add `%AZURE_KINECT_SDK%\windows-desktop\x86\release\bin` [to PATH](https://www.architectryan.com/2018/03/17/add-to-the-path-on-windows-10/)
    - should this not work for your pc: add `%AZURE_KINECT_SDK%\windows-desktop\amd64\release\bin` instead to PATH (make sure this is not set multiple times)
- camera not accessible for kinect viewer: allow desktop apps to access camera [(Start > Settings > Privacy & security > Camera)](https://support.microsoft.com/en-us/windows/manage-app-permissions-for-your-camera-in-windows-87ebc757-1f87-7bbf-84b5-0686afb6ca6b)
- `LibUSB transfer status of 00000001 unexpected` - [check the power supply of the kinect or just reconnect the power](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1335)

## build the mirror3D
1. download [visual studio 2022](https://visualstudio.microsoft.com/de/downloads/)
2. drag "project" folder to [`define_project_dir.bat`](../cgv/define_project_dir.bat)
3. drag the "build" folder to [`define_system_variables.bat`](../cgv/define_system_variables.bat) and select j for visual studio 2022
4. open the batch file [`define_platform.bat`](../cgv/define_platform.bat) and set to 64
5. bind the .pj extension to the [`generate_makefiles.bat`](../cgv/bin/generate_makefiles.bat)

## modify the cgv framework so my application works 🙈
- find the point_renderer.cxx in the cgv framework under libs/cgv_gl/point_renderer.cxx and comment the two lines:
```cpp
if (!has_point_sizes)
    ref_prog().set_attribute(ctx, "point_size", prs.point_size);
```

## things to fix on windows
- Ansii color support for Win10:
    - press <kbd>![Windows Key](https://i.stack.imgur.com/B8Zit.png)</kbd> and type `regedit`
    - HKEY_CURRENT_USER -> Console -> <kbd>Right-Click</kbd> -> New -> DWORD -> VirtualTerminalLevel
    - set this new variable to 1
- when using the glsl highlighting addon for VisualStudio2022 you might want to avoid comments like `//***** begin interface of fragment.glfs ****` and convert the to `// ***** begin interface of fragment.glfs ****` (put a space after the comment //)
