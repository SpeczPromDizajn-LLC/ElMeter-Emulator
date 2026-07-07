// (c) SpeczPromDizajn LLC, 2025
// INN/KPP 6230093924/623001001
// DUNS 362165653
// 6 Yablochkova Proezd, office 718, Ryazan, Russia
// Web: https://spd.net.ru
// e-mail: info@spd.net.ru
// Phone: +7 (4912) 52-47-14

// v. 5.1

unit Helpers;

interface

function MAKE16(a, b: Byte): UInt16;
function MAKE32(a, b, c, d: Byte): UInt32;

implementation

uses
 System.SysUtils;

function MAKE16(a, b: Byte): UInt16;
 begin
  Result := (a shl 8) or b;
 end;

function MAKE32(a, b, c, d: Byte): UInt32;
 begin
  Result := (a shl 24) or (b shl 16) or (c shl 8) or d;
 end;

end.
