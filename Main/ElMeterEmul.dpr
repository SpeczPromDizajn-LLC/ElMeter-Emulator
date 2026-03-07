program ElMeterEmul;

uses
  Vcl.Forms,
  Form_Main in 'Form_Main.pas' {frmMain},
  Vcl.Themes,
  Vcl.Styles,
  VersionInfo in 'SPDLib\VersionInfo.pas',
  Common in 'Common.pas',
  CRC in 'SPDLib\CRC.pas',
  UDialogs in 'SPDLib\UDialogs.pas',
  Class_ElMeter in 'Class_ElMeter.pas',
  Helpers in 'SPDLib\Helpers.pas',
  UStrings in 'SPDLib\UStrings.pas';

{$R *.res}

begin
  Application.Initialize;
  Application.MainFormOnTaskbar := True;
  TStyleManager.TrySetStyle('Smokey Quartz Kamri');
  Application.CreateForm(TfrmMain, frmMain);
  Application.Run;
end.
