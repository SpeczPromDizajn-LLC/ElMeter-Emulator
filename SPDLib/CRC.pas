// (c) SpeczPromDizajn LLC, 2026
// INN/KPP 6230093924/623001001
// DUNS 362165653
// 6 Yablochkova Proezd, office 718, Ryazan, Russia
// Web: https://spd.net.ru
// e-mail: info@spd.net.ru
// Phone: +7 (4912) 52-47-14

// v. 3.0

unit CRC;

interface

type
 TCRC8 = record
 private
  InitCRC: Byte;
  FCRC:    Byte;
  Polynom: Byte;
 public
  procedure Init(pInitValue, pPolynom: Byte);
  procedure StartCRC;
  procedure UpdateCRC(pValue: Byte);
  property CRC: Byte read FCRC;
 end;

 TCRC16 = record
 private
  InitCRC: UInt16;
  FCRC:    UInt16;
  Polynom: UInt16;
 public
  procedure Init(pInitValue, pPolynom: UInt16);
  procedure StartCRC;
  procedure UpdateCRC(pValue: Byte);
  property CRC: UInt16 read FCRC;
 end;

 // CRC-32/ISO-HDLC
 TCRC32 = record
 private
  FCRC: UInt32;
 public
  procedure StartCRC;
  procedure UpdateCRC(pValue: Byte);
  procedure EndCRC;
  property CRC: UInt32 read FCRC;
 end;

implementation

procedure TCRC8.Init(pInitValue, pPolynom: Byte);
 begin
  InitCRC := pInitValue;
  Polynom := pPolynom;
  StartCRC;
 end;

procedure TCRC8.StartCRC;
 begin
  FCRC := InitCRC;
 end;

procedure TCRC8.UpdateCRC(pValue: Byte);
 begin
  FCRC := FCRC xor pValue;

  for var i := 1 to 8 do
   begin
    if (FCRC and 1) > 0 then
     FCRC := (FCRC shr 1) xor Polynom
    else
     FCRC := FCRC shr 1;
   end;
 end;

procedure TCRC16.Init(pInitValue, pPolynom: UInt16);
 begin
  InitCRC := pInitValue;
  Polynom := pPolynom;
  StartCRC;
 end;

procedure TCRC16.StartCRC;
 begin
  FCRC := InitCRC;
 end;

procedure TCRC16.UpdateCRC(pValue: Byte);
 begin
  FCRC := FCRC xor pValue;

  for var i := 1 to 8 do
   if (FCRC and 1) > 0 then
    FCRC := (FCRC shr 1) xor Polynom
   else
    FCRC := FCRC shr 1;
 end;

procedure TCRC32.StartCRC;
 begin
  FCRC := $FFFFFFFF;
 end;

procedure TCRC32.EndCRC;
 begin
  FCRC := not CRC;
 end;

procedure TCRC32.UpdateCRC(pValue: Byte);
 begin
  FCRC := FCRC xor pValue;

  for var i := 1 to 8 do
   begin
    if (CRC and 1) > 0 then
     FCRC := (FCRC shr 1) xor $EDB88320
    else
     FCRC := FCRC shr 1;
   end;
 end;

end.
