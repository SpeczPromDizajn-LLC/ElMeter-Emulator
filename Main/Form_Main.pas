unit Form_Main;

interface

uses
 Winapi.Windows, Winapi.Messages, System.SysUtils, System.Variants, System.Classes, Vcl.Graphics,
 Vcl.Controls, Vcl.Forms, Vcl.Dialogs, COMPort, Vcl.StdCtrls, Vcl.ExtCtrls,
 Vcl.Imaging.pngimage, Vcl.ComCtrls, Vcl.Buttons, Vcl.Mask;

type
 TfrmMain = class(TForm)
  COM: TCOMPort;
  edtTariff5: TLabeledEdit;
  edtTariff1: TLabeledEdit;
  edtTariff3: TLabeledEdit;
  rbElMeter: TRadioGroup;
  cbCOM: TComboBox;
  Label4: TLabel;
  edtSN: TLabeledEdit;
  edtU: TLabeledEdit;
  edtI: TLabeledEdit;
  edtPwr: TLabeledEdit;
  edtTariff2: TLabeledEdit;
  edtTariff4: TLabeledEdit;
  edtFreq: TLabeledEdit;
  btnStop: TSpeedButton;
  lbLog: TListBox;
  procedure COMRxChar(Sender: TObject; Count: Integer);
  procedure FormCreate(Sender: TObject);
  procedure FormClose(Sender: TObject; var Action: TCloseAction);
  procedure rbElMeterClick(Sender: TObject);
  procedure cbCOMChange(Sender: TObject);
  procedure lbLogDrawItem(Control: TWinControl; Index: Integer; Rect: TRect; State: TOwnerDrawState);
  procedure FormActivate(Sender: TObject);
 private
 public
  procedure OpenCOM(nCOM: Integer);
  procedure AddLog(pStr: string);
 end;

var
 frmMain: TfrmMain;

implementation

{$R *.dfm}

uses
 StrUtils, VersionInfo, Common, UDialogs, Class_ElMeter, Registry, UStrings;

var
 InitCOM:     Integer      = 0;
 ElMeterType: TElMeterType = emMercury206;
 ElMeter:     TElMeter;

procedure TfrmMain.AddLog(pStr: string);
 begin
  lbLog.Items.Add(pStr);

  if lbLog.Items.Count >= 100 then
   lbLog.Items.Delete(0);

  lbLog.ItemIndex := lbLog.Items.Count - 1;
 end;

procedure TfrmMain.OpenCOM(nCOM: Integer);
 begin
  try
   COM.Close;
   COM.Port := 'COM' + IntToStr(nCOM);
   COM.Open;
  except
   MessageBoxOK(PChar(Format(STR_ERROR_OPEN_PORT, [nCOM])), PChar(Application.Title), mbtError);
  end;
 end;

procedure TfrmMain.rbElMeterClick(Sender: TObject);
 begin
  ElMeterType := TElMeterType(rbElMeter.ItemIndex);
  ElMeter.Init(ElMeterType);
 end;

procedure TfrmMain.cbCOMChange(Sender: TObject);
 var
  s: string;

 begin
  s := cbCOM.Text;
  InitCOM := StrToIntDef(Copy(s, 4, Length(s) - 3), 1);
  OpenCOM(InitCOM);
 end;

procedure TfrmMain.COMRxChar(Sender: TObject; Count: Integer);
 var
  buf:    array [0 .. 127] of Byte;
  cnt:    Integer;
  b:      Byte;
  ss, sx: string;

  function EvenCalc(pValue: Byte): Byte;
   var
    even: Byte;

   begin
    even := (pValue shr 4) or (pValue shl 4);
    even := even xor pValue;
    even := (even shr 1) xor even;
    Result := (even xor ((even and 4) shr 2)) and 1;
   end;

 begin
  cnt := Count;

  COM.Read(buf, cnt);

  if btnStop.Down then
   Exit;

  ss := '';
  sx := '';

  for var j := 0 to cnt - 1 do
   begin
    b := buf[j];

    if ElMeterType = emCE102M then
     b := b and $7F;

    ElMeter.AddRxByte(b);

    ss := ss + ' ' + IntToHex(b, 2);

    if b >= 32 then
     sx := sx + Chr(b)
    else
     sx := sx + '.';

    if Length(ss) > 100 then
     begin
      AddLog(Format('>%s'#13#10'[%s]', [ss, sx]));
      ss := '';
      sx := '';
     end;
   end;

  if ss <> '' then
   AddLog(Format('>%s'#13#10'[%s]', [ss, sx]));

  if ElMeter.RequestReceived then
   begin
    ElMeter.SN := StrToInt64Def(edtSN.Text, 0);
    ElMeter.U := Str2FloatDef(edtU.Text, 0);
    ElMeter.I := Str2FloatDef(edtI.Text, 0);
    ElMeter.Pwr := StrToIntDef(edtPwr.Text, 0);
    ElMeter.Freq := StrToFloatDef(edtFreq.Text, 0);
    ElMeter.Tariff[1] := Str2FloatDef(edtTariff1.Text, 0);
    ElMeter.Tariff[2] := Str2FloatDef(edtTariff2.Text, 0);
    ElMeter.Tariff[3] := Str2FloatDef(edtTariff3.Text, 0);
    ElMeter.Tariff[4] := Str2FloatDef(edtTariff4.Text, 0);
    ElMeter.Tariff[5] := Str2FloatDef(edtTariff5.Text, 0);

    ElMeter.CreateResponse;

    if cnt > 0 then
     begin
      ss := '';
      sx := '';

      for var j := 0 to ElMeter.BufOutCnt - 1 do
       begin
        b := ElMeter.BufOut[j];
        ss := ss + ' ' + IntToHex(b, 2);

        if b >= 32 then
         sx := sx + Chr(b)
        else
         sx := sx + '.';

        if Length(ss) > 100 then
         begin
          AddLog(Format('<%s'#13#10'[%s]', [ss, sx]));
          ss := '';
          sx := '';
         end;
       end;

      if ss <> '' then
       AddLog(Format('<%s'#13#10'[%s]', [ss, sx]));

      for var j := 0 to ElMeter.BufOutCnt - 1 do
       if ElMeterType = emCE102M then
        buf[j] := (ElMeter.BufOut[j] and $7F) or (EvenCalc(ElMeter.BufOut[j]) shl 7)
       else
        buf[j] := ElMeter.BufOut[j];

      COM.Write(buf, ElMeter.BufOutCnt);
     end;
   end;
 end;

procedure TfrmMain.FormActivate(Sender: TObject);
 const
  FirstRun: Boolean = TRUE;

 begin
  if FirstRun then
   begin
    FirstRun := FALSE;
    OpenCOM(InitCOM);
   end;
 end;

procedure TfrmMain.FormClose(Sender: TObject; var Action: TCloseAction);
 const
  IsClosed: Boolean = FALSE;

 var
  reg: TRegIniFile;

 begin
  if IsClosed then
   Exit;

  IsClosed := TRUE;

  try
   COM.Close;
  except
  end;

  reg := TRegIniFile.Create(REG_INI);
  reg.WriteInteger('Hardware', 'COM', InitCOM);
  reg.WriteInteger('Hardware', 'IdxMeter', Ord(ElMeterType));
  reg.WriteString('Params', 'SN', edtSN.Text);
  reg.WriteString('Params', 'U', edtU.Text);
  reg.WriteString('Params', 'I', edtI.Text);
  reg.WriteString('Params', 'Pwr', edtPwr.Text);
  reg.WriteString('Params', 'Freq', edtFreq.Text);
  reg.WriteString('Params', 'Tariff1', edtTariff1.Text);
  reg.WriteString('Params', 'Tariff2', edtTariff2.Text);
  reg.WriteString('Params', 'Tariff3', edtTariff3.Text);
  reg.WriteString('Params', 'Tariff4', edtTariff4.Text);
  reg.WriteString('Params', 'Tariff5', edtTariff5.Text);
  reg.Free;
 end;

procedure TfrmMain.FormCreate(Sender: TObject);
 var
  reg:     TRegIniFile;
  VerInfo: TVersionInfo;

 begin
  VerInfo := TVersionInfo.Create;
  Caption := Caption + ' v.' + VerInfo.Version.Str;
  VerInfo.Free;

  reg := TRegIniFile.Create(REG_INI);
  InitCOM := reg.ReadInteger('Hardware', 'COM', 1);
  ElMeterType := TElMeterType(reg.ReadInteger('Hardware', 'IdxMeter', 0));
  edtSN.Text := reg.ReadString('Params', 'SN', '1234');
  edtU.Text := reg.ReadString('Params', 'U', '230,15');
  edtI.Text := reg.ReadString('Params', 'I', '4,5');
  edtPwr.Text := reg.ReadString('Params', 'Pwr', '300,7');
  edtFreq.Text := reg.ReadString('Params', 'Freq', '50,5');
  edtTariff1.Text := reg.ReadString('Params', 'Tariff1', '12345');
  edtTariff2.Text := reg.ReadString('Params', 'Tariff2', '12345');
  edtTariff3.Text := reg.ReadString('Params', 'Tariff3', '12345');
  edtTariff4.Text := reg.ReadString('Params', 'Tariff4', '12345');
  edtTariff5.Text := reg.ReadString('Params', 'Tariff5', '12345');
  reg.Free;

  cbCOM.Clear;

  comEnumPorts(cbCOM.Items);

  cbCOM.OnChange := nil;
  cbCOM.ItemIndex := cbCOM.Items.IndexOf('COM' + IntToStr(InitCOM));
  cbCOM.OnChange := cbCOMChange;

  rbElMeter.OnClick := nil;
  rbElMeter.ItemIndex := Ord(ElMeterType);
  rbElMeter.OnClick := rbElMeterClick;

  ElMeter.Init(TElMeterType(ElMeterType));
 end;

procedure TfrmMain.lbLogDrawItem(Control: TWinControl; Index: Integer; Rect: TRect; State: TOwnerDrawState);
 var
  sl: TArray<string>;

 begin
  with lbLog.Canvas do
   begin
    sl := lbLog.Items[Index].Split([#13#10]);

    if lbLog.Items[Index][1] = '>' then
     Font.Color := clBlack
    else
     Font.Color := clBlue;

    FillRect(Rect);
    TextOut(Rect.Left, Rect.Top, Copy(sl[0], 2, Length(sl[0]) - 1));
    TextOut(Rect.Left, Rect.Top + 13, sl[1]);
   end;
 end;

end.
