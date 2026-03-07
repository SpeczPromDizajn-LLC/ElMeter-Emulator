unit Class_ElMeter;

interface

const
 EM_METER_TIMEOUT  = 300;
 EM_METER_BUF_SIZE = 128;

 EM_MERCURY_CMD_GET_ENERGY    = $27;
 EM_MERCURY_CMD_GET_PWR_STATE = $63;
 EM_MERCURY_CMD_GET_FREQ      = $81;

 EM_CE102_MASTER_ADDR = 253;
 EM_CE102_MASTER_PSW  = 777777;
 EM_CE102_TARIFF_ADDR = $0130;
 EM_CE102_SN_ADDR     = $011A;
 EM_CE102_END         = $C0;
 EM_CE102_ESC         = $DB;
 EM_CE102_POLYNOM     = $B5;

 EM_STAR104_START                   = $73;
 EM_STAR104_STOP                    = $55;
 EM_STAR104_MASTER_ADDR             = $FFFF;
 EM_STAR104_CMD_READ_STATUS_COUNTER = $05;
 EM_STAR104_CMD_READ_FACTORY_STRING = $0A;
 EM_STAR104_CMD_READ_INSTANT_VALUE  = $2B;
 EM_STAR104_POLYNOM                 = $A9;

type
 TElMeterType = (emMercury206, emCE102, emCE102M, emStar104);

 TElMEterBuf = array [0 .. EM_METER_BUF_SIZE - 1] of Byte;

 TElMeter = record
 private
  Typ:              TElMeterType;
  fSN:              UInt64;
  fU:               UInt16;                   // x 10
  fI:               UInt32;                   // x 100
  fPwr:             UInt32;                   // x 1
  fFreq:            UInt16;                   // x 100
  fTariff:          array [1 .. 5] of UInt32; // x 100
  fBufIn:           TElMEterBuf;
  fBufInCnt:        Integer;
  fBufOut:          TElMEterBuf;
  fBufOutCnt:       Integer;
  fRequestReceived: Boolean;
  Ticks:            Int64;
  ReqSN:            UInt32;
  ReqIdx:           Integer;
  ReqCmd:           UInt16;
  ReqCmdStr:        string;
  Staff:            Boolean;
  ce102mCRC:        Byte;
  crc8:             Byte;
  crc8Polynom:      Byte;
  procedure AddByteBufIn(pValue: Byte);
  procedure AddByteBufOut(pValue: Byte);
  procedure SetU(pValue: Double);
  procedure SetI(pValue: Double);
  procedure SetFreq(pValue: Double);
  procedure SetTariff(pIdx: Integer; pValue: Double);
  procedure crc8Init(pPolynom: Byte);
  procedure crc8Update(pValue: Byte);
  procedure ce102DecodeByte(pValue: Byte);
  procedure ce102mAddStrBufOut(pStr: string);
  procedure ce102AddByteBufOut(pValue: Byte);
  procedure star104AddByteBufOut(pValue: Byte);
  procedure star104DecodeByte(pValue: Byte);
  procedure AddRxByteMercury206(pValue: Byte);
  procedure AddRxByteCE102M(pValue: Byte);
  procedure AddRxByteCE102(pValue: Byte);
  procedure AddRxByteStar104(pValue: Byte);
  procedure CreateResponseMercury206;
  procedure CreateResponseCE102M;
  procedure CreateResponseCE102;
  procedure CreateResponseStar104;
 public
  procedure Init(pTyp: TElMeterType);
  procedure AddRxByte(pValue: Byte);
  procedure CreateResponse;

  property RequestReceived: Boolean read fRequestReceived;
  property SN: UInt64 write fSN;
  property U: Double write SetU;
  property I: Double write SetI;
  property Pwr: UInt32 write fPwr;
  property Freq: Double write SetFreq;
  property Tariff[idx: Integer]: Double write SetTariff;
  property BufOut: TElMEterBuf read fBufOut;
  property BufOutCnt: Integer read fBufOutCnt;
 end;

implementation

uses
 SysUtils, StrUtils, Windows, CRC, UStrings, Helpers;

procedure TElMeter.AddByteBufIn(pValue: Byte);
 begin
  fBufIn[fBufInCnt] := pValue;
  Inc(fBufInCnt);
 end;

procedure TElMeter.AddByteBufOut(pValue: Byte);
 begin
  fBufOut[fBufOutCnt] := pValue;
  Inc(fBufOutCnt);
 end;

procedure TElMeter.crc8Init;
 begin
  crc8 := 0;
  crc8Polynom := pPolynom;
 end;

procedure TElMeter.crc8Update(pValue: Byte);
 begin
  crc8 := crc8 xor pValue;

  for var j := 1 to 8 do
   begin
    if (crc8 and 128) > 0 then
     crc8 := (crc8 shl 1) xor crc8Polynom
    else
     crc8 := crc8 shl 1;
   end;
 end;

procedure TElMeter.ce102DecodeByte(pValue: Byte);
 begin
  case pValue of
   EM_CE102_END:
    Staff := FALSE;

   $DB:
    Staff := TRUE;

   $DD:
    if Staff then
     begin
      pValue := EM_CE102_ESC;
      Staff := FALSE;
     end;

   $DC:
    if Staff then
     begin
      pValue := EM_CE102_END;
      Staff := FALSE;
     end;
  end;

  if not Staff then
   AddByteBufIn(pValue);
 end;

procedure TElMeter.ce102mAddStrBufOut(pStr: string);
 var
  b: Byte;

 begin
  for var j := 1 to Length(pStr) do
   begin
    b := Ord(pStr[j]);

    AddByteBufOut(b);

    if j > 1 then
     Inc(ce102mCRC, b);
   end;
 end;

procedure TElMeter.ce102AddByteBufOut(pValue: Byte);
 begin
  crc8Update(pValue);

  case pValue of
   EM_CE102_END:
    begin
     AddByteBufOut($DB);
     AddByteBufOut($DC);
    end;

   EM_CE102_ESC:
    begin
     AddByteBufOut($DB);
     AddByteBufOut($DD);
    end;

  else
   AddByteBufOut(pValue);
  end;
 end;

procedure TElMeter.star104AddByteBufOut(pValue: Byte);
 begin
  crc8Update(pValue);

  case pValue of
   EM_STAR104_START:
    begin
     AddByteBufOut(EM_STAR104_START);
     AddByteBufOut($22);
    end;

   EM_STAR104_STOP:
    begin
     AddByteBufOut(EM_STAR104_START);
     AddByteBufOut($11);
    end;

  else
   AddByteBufOut(pValue);
  end;
 end;

procedure TElMeter.star104DecodeByte(pValue: Byte);
 begin
  case pValue of
   EM_STAR104_STOP:
    begin
     if Staff then
      AddByteBufIn(EM_STAR104_START);

     Staff := FALSE;
    end;

   EM_STAR104_START:
    Staff := TRUE;

   $22:
    if Staff then
     begin
      pValue := EM_STAR104_START;
      Staff := FALSE;
     end;

   $11:
    if Staff then
     begin
      pValue := EM_STAR104_STOP;
      Staff := FALSE;
     end;
  end;

  if not Staff then
   AddByteBufIn(pValue);
 end;

procedure TElMeter.SetU(pValue: Double);
 begin
  if Typ = emStar104 then
   fU := Round(pValue * 100)
  else
   fU := Round(pValue * 10);
 end;

procedure TElMeter.SetI(pValue: Double);
 begin
  if Typ = emStar104 then
   fI := Round(pValue * 1000)
  else
   fI := Round(pValue * 100);
 end;

procedure TElMeter.SetFreq(pValue: Double);
 begin
  fFreq := Round(pValue * 100);
 end;

procedure TElMeter.SetTariff(pIdx: Integer; pValue: Double);
 begin
  if (pIdx < 1) or (pIdx > 5) then
   Exit;

  fTariff[pIdx] := Round(pValue * 100);
 end;

procedure TElMeter.AddRxByteMercury206(pValue: Byte);
 var
  CRC: TCRC16;

 begin
  AddByteBufIn(pValue);

  if fBufInCnt = 7 then
   begin
    CRC.Init($FFFF, $A001);

    for var j := 0 to 4 do
     CRC.UpdateCRC(fBufIn[j]);

    if CRC.CRC = MAKE16(fBufIn[6], fBufIn[5]) then
     begin
      ReqSN := MAKE32(fBufIn[0], fBufIn[1], fBufIn[2], fBufIn[3]);
      ReqCmd := fBufIn[4];
      fBufInCnt := 0;
      fRequestReceived := TRUE;
     end;
   end;
 end;

procedure TElMeter.AddRxByteCE102M(pValue: Byte);
 var
  s:         string;
  crc8:      Byte;
  validCRC8: Boolean;

 begin
  AddByteBufIn(pValue);

  s := '';
  crc8 := 0;

  for var j := 0 to fBufInCnt - 1 do
   s := s + Chr(fBufIn[j]);

  for var j := 1 to fBufInCnt - 2 do
   Inc(crc8, fBufIn[j]);

  validCRC8 := (crc8 and $7F) = fBufIn[fBufInCnt - 1];

  if Pos('/?!'#13#10, s) > 0 then
   begin
    ReqCmdStr := '/?!';
    fRequestReceived := TRUE;
   end;

  if Pos(#6'051'#13#10, s) > 0 then
   begin
    ReqCmdStr := '051';
    fRequestReceived := TRUE;
   end;

  if (Pos('VOLTA()', s) > 0) and validCRC8 then
   begin
    ReqCmdStr := 'VOLTA';
    fRequestReceived := TRUE;
   end;

  if (Pos('CURRE()', s) > 0) and validCRC8 then
   begin
    ReqCmdStr := 'CURRE';
    fRequestReceived := TRUE;
   end;

  if (Pos('POWEP()', s) > 0) and validCRC8 then
   begin
    ReqCmdStr := 'POWEP';
    fRequestReceived := TRUE;
   end;

  if (Pos('FREQU()', s) > 0) and validCRC8 then
   begin
    ReqCmdStr := 'FREQU';
    fRequestReceived := TRUE;
   end;

  if (Pos('R1'#2'ET0PE(02)', s) > 0) and validCRC8 then
   begin
    ReqCmdStr := 'T1';
    fRequestReceived := TRUE;
   end;

  if (Pos('R1'#2'ET0PE(03)', s) > 0) and validCRC8 then
   begin
    ReqCmdStr := 'T2';
    fRequestReceived := TRUE;
   end;

  if (Pos('R1'#2'ET0PE(04)', s) > 0) and validCRC8 then
   begin
    ReqCmdStr := 'T3';
    fRequestReceived := TRUE;
   end;

  if (Pos('R1'#2'ET0PE(05)', s) > 0) and validCRC8 then
   begin
    ReqCmdStr := 'T4';
    fRequestReceived := TRUE;
   end;
 end;

procedure TElMeter.AddRxByteCE102(pValue: Byte);
 begin
  ce102DecodeByte(pValue);

  if fBufInCnt > 2 then
   if (fBufIn[0] = EM_CE102_END) and (fBufIn[fBufInCnt - 1] = EM_CE102_END) then
    begin
     crc8Init(EM_CE102_POLYNOM);

     for var j := 1 to fBufInCnt - 3 do
      crc8Update(fBufIn[j]);

     if crc8 = fBufIn[fBufInCnt - 2] then
      begin
       ReqSN := MAKE16(fBufIn[3], fBufIn[2]);
       ReqCmd := MAKE16(fBufIn[11], fBufIn[12]);

       if ReqCmd = EM_CE102_TARIFF_ADDR then
        ReqIdx := fBufIn[14]
       else
        ReqIdx := fBufIn[13];

       fRequestReceived := TRUE;
      end;

     fBufInCnt := 0;
    end;
 end;

procedure TElMeter.AddRxByteStar104(pValue: Byte);
 begin
  star104DecodeByte(pValue);

  if fBufInCnt >= 3 then
   if (fBufIn[0] = EM_STAR104_START) and ((fBufIn[1] = EM_STAR104_STOP)) and (fBufIn[fBufInCnt - 1] = EM_STAR104_STOP) then
    begin
     crc8Init(EM_STAR104_POLYNOM);

     for var j := 2 to fBufInCnt - 3 do
      crc8Update(fBufIn[j]);

     if crc8 = fBufIn[fBufInCnt - 2] then
      begin
       ReqSN := MAKE16(fBufIn[5], fBufIn[4]);
       ReqCmd := fBufIn[8];
       fRequestReceived := TRUE;
      end;

     fBufInCnt := 0;
    end;
 end;

procedure TElMeter.CreateResponseMercury206;
 var
  CRC: TCRC16;

 begin
  case ReqCmd of
   EM_MERCURY_CMD_GET_PWR_STATE:
    begin
     AddByteBufOut(ReqSN shr 24);
     AddByteBufOut((ReqSN shr 16) and $FF);
     AddByteBufOut((ReqSN shr 8) and $FF);
     AddByteBufOut(ReqSN and $FF);

     AddByteBufOut(ReqCmd);

     AddByteBufOut(BIN2BCD(fU div 100));
     AddByteBufOut(BIN2BCD(fU mod 100));

     AddByteBufOut(BIN2BCD(fI div 100));
     AddByteBufOut(BIN2BCD(fI mod 100));

     AddByteBufOut(BIN2BCD(fPwr div 10000));
     AddByteBufOut(BIN2BCD((fPwr div 100) mod 100));
     AddByteBufOut(BIN2BCD(fPwr mod 100));
    end;

   EM_MERCURY_CMD_GET_ENERGY:
    begin
     AddByteBufOut(ReqSN shr 24);
     AddByteBufOut((ReqSN shr 16) and $FF);
     AddByteBufOut((ReqSN shr 8) and $FF);
     AddByteBufOut(ReqSN and $FF);

     AddByteBufOut(ReqCmd);

     for var k := 1 to 4 do
      begin
       AddByteBufOut(BIN2BCD((fTariff[k] div 1000000) mod 100));
       AddByteBufOut(BIN2BCD((fTariff[k] div 10000) mod 100));
       AddByteBufOut(BIN2BCD((fTariff[k] div 100) mod 100));
       AddByteBufOut(BIN2BCD(fTariff[k] mod 100));
      end;
    end;

   EM_MERCURY_CMD_GET_FREQ:
    begin
     AddByteBufOut(ReqSN shr 24);
     AddByteBufOut((ReqSN shr 16) and $FF);
     AddByteBufOut((ReqSN shr 8) and $FF);
     AddByteBufOut(ReqSN and $FF);

     AddByteBufOut(ReqCmd);

     AddByteBufOut(BIN2BCD(fFreq div 100));
     AddByteBufOut(BIN2BCD(fFreq mod 100));

     AddByteBufOut($3A);

     AddByteBufOut($00);
     AddByteBufOut($00);
     AddByteBufOut($00);
     AddByteBufOut($00);
     AddByteBufOut($00);
     AddByteBufOut($00);
    end;
  end;

  if fBufOutCnt > 0 then
   begin
    CRC.Init($FFFF, $A001);

    for var j := 0 to fBufOutCnt - 1 do
     CRC.UpdateCRC(fBufOut[j]);

    AddByteBufOut(CRC.CRC and $FF);
    AddByteBufOut(CRC.CRC shr 8);
   end;
 end;

procedure TElMeter.CreateResponseCE102M;
 begin
  ce102mCRC := 0;

  if ReqCmdStr = '/?!' then
   ce102mAddStrBufOut('/EKT5CE102Mv01'#13#10);

  if ReqCmdStr = '051' then
   begin
    ce102mAddStrBufOut(Format(#1'P0'#2'(%d)'#3, [fSN]));
    AddByteBufOut(ce102mCRC and $7F);
   end;

  if ReqCmdStr = 'VOLTA' then
   begin
    ce102mAddStrBufOut(Format(#2'VOLTA(%0.1f)'#13#10#3, [fU / 10]));
    AddByteBufOut(ce102mCRC and $7F);
   end;

  if ReqCmdStr = 'CURRE' then
   begin
    ce102mAddStrBufOut(Format(#2'CURRE(%0.2f)'#13#10#3, [fI / 100]));
    AddByteBufOut(ce102mCRC and $7F);
   end;

  if ReqCmdStr = 'POWEP' then
   begin
    ce102mAddStrBufOut(Format(#2'POWEP(%0.3f)'#13#10#3, [fPwr / 1000]));
    AddByteBufOut(ce102mCRC and $7F);
   end;

  if ReqCmdStr = 'FREQU' then
   begin
    ce102mAddStrBufOut(Format(#2'FREQU(%0.2f)'#13#10#3, [fFreq / 100]));
    AddByteBufOut(ce102mCRC and $7F);
   end;

  if ReqCmdStr = 'T1' then
   begin
    ce102mAddStrBufOut(Format(#2'ET0PE(%0.2f)'#13#10#3, [fTariff[1] / 100]));
    AddByteBufOut(ce102mCRC and $7F);
   end;

  if ReqCmdStr = 'T2' then
   begin
    ce102mAddStrBufOut(Format(#2'ET0PE(%0.2f)'#13#10#3, [fTariff[2] / 100]));
    AddByteBufOut(ce102mCRC and $7F);
   end;

  if ReqCmdStr = 'T3' then
   begin
    ce102mAddStrBufOut(Format(#2'ET0PE(%0.2f)'#13#10#3, [fTariff[3] / 100]));
    AddByteBufOut(ce102mCRC and $7F);
   end;

  if ReqCmdStr = 'T4' then
   begin
    ce102mAddStrBufOut(Format(#2'ET0PE(%0.2f)'#13#10#3, [fTariff[4] / 100]));
    AddByteBufOut(ce102mCRC and $7F);
   end;
 end;

procedure TElMeter.CreateResponseCE102;
 var
  day, month, year: UInt16;

 begin
  DecodeDate(Now, year, month, day);

  crc8Init(EM_CE102_POLYNOM);
  fBufOutCnt := 0;

  case ReqCmd of
   EM_CE102_TARIFF_ADDR:
    begin
     AddByteBufOut(EM_CE102_END);

     ce102AddByteBufOut($48);
     ce102AddByteBufOut(EM_CE102_MASTER_ADDR and $FF);
     ce102AddByteBufOut((EM_CE102_MASTER_ADDR shr 8) and $FF);
     ce102AddByteBufOut(ReqSN and $FF);
     ce102AddByteBufOut((ReqSN shr 8) and $FF);
     ce102AddByteBufOut($57);
     ce102AddByteBufOut((EM_CE102_TARIFF_ADDR shr 8) and $FF);
     ce102AddByteBufOut(EM_CE102_TARIFF_ADDR and $FF);
     ce102AddByteBufOut(BIN2BCD(day));
     ce102AddByteBufOut(BIN2BCD(month));
     ce102AddByteBufOut(BIN2BCD(year mod 100));

     ce102AddByteBufOut(fTariff[ReqIdx] and $FF);
     ce102AddByteBufOut((fTariff[ReqIdx] shr 8) and $FF);
     ce102AddByteBufOut((fTariff[ReqIdx] shr 16) and $FF);
     ce102AddByteBufOut(fTariff[ReqIdx] shr 24);

     ce102AddByteBufOut(crc8);
     AddByteBufOut(EM_CE102_END);
    end;

   EM_CE102_SN_ADDR:
    begin
     AddByteBufOut(EM_CE102_END);

     ce102AddByteBufOut($48);
     ce102AddByteBufOut(EM_CE102_MASTER_ADDR and $FF);
     ce102AddByteBufOut((EM_CE102_MASTER_ADDR shr 8) and $FF);
     ce102AddByteBufOut(ReqSN and $FF);
     ce102AddByteBufOut((ReqSN shr 8) and $FF);
     ce102AddByteBufOut($58);
     ce102AddByteBufOut((EM_CE102_SN_ADDR shr 8) and $FF);
     ce102AddByteBufOut(EM_CE102_SN_ADDR and $FF);

     if ReqIdx = 0 then
      begin
       ce102AddByteBufOut($30 + (fSN mod 10));
       ce102AddByteBufOut($30 + ((fSN div 10) mod 10));
       ce102AddByteBufOut($30 + ((fSN div 100) mod 10));
       ce102AddByteBufOut($30 + ((fSN div 1000) mod 10));
       ce102AddByteBufOut($30 + ((fSN div 10000) mod 10));
       ce102AddByteBufOut($30 + ((fSN div 100000) mod 10));
       ce102AddByteBufOut($30 + ((fSN div 1000000) mod 10));
       ce102AddByteBufOut($30 + ((fSN div 10000000) mod 10));
      end
     else
      begin
       ce102AddByteBufOut($30 + ((fSN div 100000000) mod 10));
       ce102AddByteBufOut($30 + ((fSN div 1000000000) mod 10));
       ce102AddByteBufOut($30 + ((fSN div 10000000000) mod 10));
       ce102AddByteBufOut($30 + ((fSN div 100000000000) mod 10));
       ce102AddByteBufOut($30 + ((fSN div 1000000000000) mod 10));
       ce102AddByteBufOut($30 + ((fSN div 10000000000000) mod 10));
       ce102AddByteBufOut($30 + ((fSN div 100000000000000) mod 10));
       ce102AddByteBufOut($00);
      end;

     ce102AddByteBufOut(crc8);

     AddByteBufOut(EM_CE102_END);
    end;
  end;
 end;

procedure TElMeter.CreateResponseStar104;
 var
  s: string;

 begin
  crc8Init(EM_STAR104_POLYNOM);
  fBufOutCnt := 0;

  case ReqCmd of
   EM_STAR104_CMD_READ_INSTANT_VALUE:
    begin
     AddByteBufOut(EM_STAR104_START);
     AddByteBufOut(EM_STAR104_STOP);

     star104AddByteBufOut($1C);
     star104AddByteBufOut($00);
     star104AddByteBufOut(EM_STAR104_MASTER_ADDR and $FF);
     star104AddByteBufOut((EM_STAR104_MASTER_ADDR shr 8) and $FF);
     star104AddByteBufOut(ReqSN and $FF);
     star104AddByteBufOut((ReqSN shr 8) and $FF);
     star104AddByteBufOut(EM_STAR104_CMD_READ_INSTANT_VALUE);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);

     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut(fPwr and $FF);
     star104AddByteBufOut(fPwr shr 8);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut(fFreq and $FF);
     star104AddByteBufOut(fFreq shr 8);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut(fU and $FF);
     star104AddByteBufOut(fU shr 8);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut(fI and $FF);
     star104AddByteBufOut((fI shr 8) and $FF);
     star104AddByteBufOut((fI shr 16) and $FF);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);

     star104AddByteBufOut(crc8);

     AddByteBufOut(EM_STAR104_STOP);
    end;

   EM_STAR104_CMD_READ_FACTORY_STRING:
    begin
     AddByteBufOut(EM_STAR104_START);
     AddByteBufOut(EM_STAR104_STOP);

     star104AddByteBufOut($1C);
     star104AddByteBufOut($00);
     star104AddByteBufOut(EM_STAR104_MASTER_ADDR and $FF);
     star104AddByteBufOut((EM_STAR104_MASTER_ADDR shr 8) and $FF);
     star104AddByteBufOut(ReqSN and $FF);
     star104AddByteBufOut((ReqSN shr 8) and $FF);
     star104AddByteBufOut(EM_STAR104_CMD_READ_FACTORY_STRING);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);

     star104AddByteBufOut($00);
     s := IntToStr(fSN);

     for var k := 1 to Length(s) do
      star104AddByteBufOut(Ord(s[k]));

     star104AddByteBufOut($00);

     star104AddByteBufOut(crc8);

     AddByteBufOut(EM_STAR104_STOP);
    end;

   EM_STAR104_CMD_READ_STATUS_COUNTER:
    begin
     AddByteBufOut(EM_STAR104_START);
     AddByteBufOut(EM_STAR104_STOP);

     star104AddByteBufOut($1C);
     star104AddByteBufOut($00);
     star104AddByteBufOut(EM_STAR104_MASTER_ADDR and $FF);
     star104AddByteBufOut((EM_STAR104_MASTER_ADDR shr 8) and $FF);
     star104AddByteBufOut(ReqSN and $FF);
     star104AddByteBufOut((ReqSN shr 8) and $FF);
     star104AddByteBufOut(EM_STAR104_CMD_READ_STATUS_COUNTER);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);

     star104AddByteBufOut($00);
     star104AddByteBufOut($02);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut($00);
     star104AddByteBufOut(fTariff[1] and $FF);
     star104AddByteBufOut((fTariff[1] shr 8) and $FF);
     star104AddByteBufOut((fTariff[1] shr 16) and $FF);
     star104AddByteBufOut(fTariff[1] shr 24);

     star104AddByteBufOut(fTariff[2] and $FF);
     star104AddByteBufOut((fTariff[2] shr 8) and $FF);
     star104AddByteBufOut((fTariff[2] shr 16) and $FF);
     star104AddByteBufOut(fTariff[2] shr 24);

     star104AddByteBufOut(fTariff[3] and $FF);
     star104AddByteBufOut((fTariff[3] shr 8) and $FF);
     star104AddByteBufOut((fTariff[3] shr 16) and $FF);
     star104AddByteBufOut(fTariff[3] shr 24);

     star104AddByteBufOut(fTariff[4] and $FF);
     star104AddByteBufOut((fTariff[4] shr 8) and $FF);
     star104AddByteBufOut((fTariff[4] shr 16) and $FF);
     star104AddByteBufOut(fTariff[4] shr 24);

     star104AddByteBufOut(crc8);

     AddByteBufOut(EM_STAR104_STOP);
    end;
  end;
 end;

procedure TElMeter.Init(pTyp: TElMeterType);
 begin
  Typ := pTyp;
  fSN := 0;
  fU := 0;
  fI := 0;
  fPwr := 0;
  fFreq := 0;
  fTariff[1] := 0;
  fTariff[2] := 0;
  fTariff[3] := 0;
  fTariff[4] := 0;
  fTariff[5] := 0;

  fBufInCnt := 0;
  fBufOutCnt := 0;

  fRequestReceived := FALSE;
  Ticks := GetTickCount64;

  Staff := FALSE;
 end;

procedure TElMeter.AddRxByte(pValue: Byte);
 begin
  if Abs(GetTickCount64 - Ticks) > EM_METER_TIMEOUT then
   fBufInCnt := 0;

  Ticks := GetTickCount64;

  if fBufInCnt >= EM_METER_BUF_SIZE then
   Exit;

  case Typ of
   emMercury206:
    AddRxByteMercury206(pValue);

   emCE102M:
    AddRxByteCE102M(pValue);

   emCE102:
    AddRxByteCE102(pValue);

   emStar104:
    AddRxByteStar104(pValue);
  end;
 end;

procedure TElMeter.CreateResponse;
 begin
  fBufOutCnt := 0;

  if not fRequestReceived then
   Exit;

  fRequestReceived := FALSE;

  if (Typ = emMercury206) and (ReqSN <> fSN) then
   Exit;

  if (Typ = emCE102) and (ReqSN <> (fSN mod 100000)) then
   Exit;

  case Typ of
   emMercury206:
    CreateResponseMercury206;

   emCE102M:
    CreateResponseCE102M;

   emCE102:
    CreateResponseCE102;

   emStar104:
    CreateResponseStar104;
  end;
 end;

end.
