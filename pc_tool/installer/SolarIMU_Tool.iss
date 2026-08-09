#define MyAppName "Solar IMU 路牌监测工具"
#define MyAppVersion "1.1.1"
#define MyAppPublisher "Solar IMU"
#define MyAppExeName "SolarIMU_Tool.exe"

[Setup]
AppId={{A15FB384-D400-46E3-97BB-7BE51AC88DEB}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppVerName={#MyAppName} V{#MyAppVersion}
AppPublisher={#MyAppPublisher}
DefaultDirName={localappdata}\Programs\SolarIMU_Tool
DefaultGroupName={#MyAppName}
DisableProgramGroupPage=yes
PrivilegesRequired=lowest
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
OutputDir=..\release_dist
OutputBaseFilename=SolarIMU_Tool_Setup_v1.1.1_Win64
Compression=lzma2/ultra64
SolidCompression=yes
WizardStyle=modern
SetupLogging=yes
CloseApplications=yes
RestartApplications=no
UninstallDisplayIcon={app}\{#MyAppExeName}
UninstallDisplayName=卸载 {#MyAppName}
VersionInfoVersion=1.1.1.0
VersionInfoProductName={#MyAppName}
VersionInfoProductVersion={#MyAppVersion}
VersionInfoDescription={#MyAppName} 安装程序

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "创建桌面快捷方式"; GroupDescription: "附加任务："

[Files]
Source: "..\release_dist\SolarIMU_Tool\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{autoprograms}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"
Name: "{autoprograms}\卸载 {#MyAppName}"; Filename: "{uninstallexe}"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: desktopicon
Name: "{app}\卸载 {#MyAppName}"; Filename: "{uninstallexe}"

[Run]
Filename: "{app}\{#MyAppExeName}"; Description: "启动 {#MyAppName}"; Flags: nowait postinstall skipifsilent
