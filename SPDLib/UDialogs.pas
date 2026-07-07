// (c) SpeczPromDizajn LLC, 2021
// INN/KPP 6230093924/623001001
// DUNS 362165653
// 6 Yablochkova Proezd, office 718, Ryazan, Russia
// Web: https://spd.net.ru
// e-mail: info@spd.net.ru
// Phone: +7 (4912) 52-47-14

// v. 5.0

unit UDialogs;

interface

uses
 Vcl.Dialogs, Vcl.Controls, Vcl.Forms;

type
 TMBType    = (mbtNone, mbtWarning, mbtError, mbtInformation, mbtShield);
 TMBResType = (mbrYes, mbrNo, mbrCancel);

procedure MessageBoxOK(pTitle, pText: string; pType: TMBType);
function MessageBoxYesNo(pTitle, pText: string; pType: TMBType; pDefaulButton: TTaskDialogCommonButton = tcbNo): TMBResType;
function MessageBoxYesNoCancel(pTitle, pText: string; pType: TMBType; pDefaulButton: TTaskDialogCommonButton = tcbCancel): TMBResType;
function GetProgramName: string;

implementation

uses
 System.IOutils, Winapi.Windows;

procedure MessageBoxOK(pTitle, pText: string; pType: TMBType);
 var
  td: TTaskDialog;

 begin
  td := TTaskDialog.Create(Application);
  td.Buttons.Clear;
  td.RadioButtons.Clear;
  td.MainIcon := Ord(pType);
  td.Caption := Application.Title;
  td.Title := pTitle;
  td.Text := pText;
  td.CommonButtons := [tcbOK];
  td.Execute;
  td.Free;
 end;

function MessageBoxYesNo(pTitle, pText: string; pType: TMBType; pDefaulButton: TTaskDialogCommonButton = tcbNo): TMBResType;
 var
  td: TTaskDialog;

 begin
  td := TTaskDialog.Create(Application);
  td.Buttons.Clear;
  td.RadioButtons.Clear;
  td.MainIcon := Ord(pType);
  td.Caption := Application.Title;
  td.Title := pTitle;
  td.Text := pText;
  td.CommonButtons := [tcbYes, tcbNo];
  td.DefaultButton := pDefaulButton;
  td.Execute;

  if td.ModalResult = ID_Yes then
   Result := mbrYes
  else
   Result := mbrNo;

  td.Free;
 end;

function MessageBoxYesNoCancel(pTitle, pText: string; pType: TMBType; pDefaulButton: TTaskDialogCommonButton = tcbCancel): TMBResType;
 var
  td: TTaskDialog;

 begin
  td := TTaskDialog.Create(Application);
  td.Buttons.Clear;
  td.RadioButtons.Clear;
  td.MainIcon := Ord(pType);
  td.Caption := Application.Title;
  td.Title := pTitle;
  td.Text := pText;
  td.CommonButtons := [tcbYes, tcbNo, tcbCancel];
  td.DefaultButton := pDefaulButton;
  td.Execute;

  case td.ModalResult of
   ID_Yes:
    Result := mbrYes;

   ID_No:
    Result := mbrNo;

  else
   Result := mbrCancel;
  end;

  td.Free;
 end;

function GetProgramName: string;
 begin
  Result := TPath.GetFileNameWithoutExtension(ParamStr(0));
 end;

end.
