# ============================================================================
#  xinput_probe.ps1 -- Windows-side verdict probe for the XInput device the
#    Jetson emulates (VID 0x045E / PID 0x028E, wired Xbox 360 pad).
#
#  Mechanism: P/Invoke xinput1_4.dll!XInputGetState, polling the four user
#    slots and printing the return code, the packet number and the decoded
#    report. rc = 1167 (ERROR_DEVICE_NOT_CONNECTED) = slot empty; rc = 0 with
#    a growing packet number = live device. The packet number is the host
#    input buffer's own counter, so pressing a button on the physical pad
#    (or driving a virtual one) changes both the packet number and the
#    decoded button/axis fields.
#
#  Usage (no admin needed):
#    powershell -ExecutionPolicy Bypass -File scripts/test/xinput_probe.ps1
#    powershell ... -File scripts/test/xinput_probe.ps1 -Count 40 -IntervalMs 100
#    powershell ... -File scripts/test/xinput_probe.ps1 -Slot 0 -Once
#  Exit code 0 = at least one slot reported rc = 0, 1 = no live slot.
#  ASCII only on purpose: Windows PowerShell 5.1 parses a BOM-less script in
#    its ANSI codepage, so non-ASCII source would need a BOM.
# ============================================================================
[CmdletBinding()]
param(
    [int]$Count = 8,
    [int]$IntervalMs = 250,
    [int]$Slot = -1,        # -1 = poll all four slots, 0..3 = one slot
    [switch]$Once           # single pass, ignore -Count/-IntervalMs
)

$ErrorActionPreference = 'Stop'

Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;

namespace XInputProbe
{
    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct XINPUT_GAMEPAD
    {
        public ushort wButtons;
        public byte bLeftTrigger;
        public byte bRightTrigger;
        public short sThumbLX;
        public short sThumbLY;
        public short sThumbRX;
        public short sThumbRY;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct XINPUT_STATE
    {
        public uint dwPacketNumber;
        public XINPUT_GAMEPAD Gamepad;
    }

    public static class Native
    {
        [DllImport("xinput1_4.dll", EntryPoint = "XInputGetState")]
        public static extern uint XInputGetState(uint dwUserIndex, out XINPUT_STATE pState);
    }
}
'@

# XInput button bits (xinput.h). The GUIDE bit (0x0400) is not reported by
# XInputGetState -- the guide button is exposed through XInputGetKeystroke only.
# Pairs are (bit, name); GetEnumerator() is used so the numeric keys never turn
# into string keys of the ordered dictionary.
$ButtonBits = @(
    @(0x0001, 'UP'),    @(0x0002, 'DOWN'),  @(0x0004, 'LEFT'), @(0x0008, 'RIGHT')
    @(0x0010, 'START'), @(0x0020, 'BACK'),  @(0x0040, 'L3'),   @(0x0080, 'R3')
    @(0x0100, 'LB'),    @(0x0200, 'RB'),    @(0x1000, 'A')
    @(0x2000, 'B'),     @(0x4000, 'X'),     @(0x8000, 'Y')
)
$RC_EMPTY = 1167    # ERROR_DEVICE_NOT_CONNECTED

function Format-Buttons([int]$bits) {
    if ($bits -eq 0) { return 'none' }
    $names = @()
    foreach ($b in $ButtonBits) { if ($bits -band $b[0]) { $names += $b[1] } }
    return ($names -join '+')
}

$slots = if ($Slot -ge 0) { @($Slot) } else { @(0, 1, 2, 3) }
$stat = @{}
foreach ($s in $slots) { $stat[$s] = [pscustomobject]@{ Rc = $null; First = $null; Last = $null; Live = $false } }

$passes = if ($Once) { 1 } else { $Count }
$t0 = Get-Date
Write-Output ("xinput_probe: slots=[{0}] passes={1} intervalMs={2} xinput1_4 = {3}" -f `
    ($slots -join ','), $passes, $IntervalMs, (Join-Path $env:SystemRoot 'System32\xinput1_4.dll'))
for ($p = 1; $p -le $passes; $p++) {
    foreach ($s in $slots) {
        $st = New-Object XInputProbe.XINPUT_STATE
        $rc = [XInputProbe.Native]::XInputGetState([uint32]$s, [ref]$st)
        $g = $st.Gamepad
        if ($rc -eq 0) {
            $stat[$s].Live = $true
            if ($null -eq $stat[$s].First) { $stat[$s].First = $st.dwPacketNumber }
            $stat[$s].Last = $st.dwPacketNumber
            Write-Output ("[{0,3}] slot{1} rc=0 pkt={2} btns=0x{3:X4}({4}) LT={5} RT={6} LX={7} LY={8} RX={9} RY={10}" -f `
                $p, $s, $st.dwPacketNumber, $g.wButtons, (Format-Buttons $g.wButtons), `
                $g.bLeftTrigger, $g.bRightTrigger, $g.sThumbLX, $g.sThumbLY, $g.sThumbRX, $g.sThumbRY)
        } elseif ($rc -eq $RC_EMPTY) {
            $stat[$s].Rc = $rc
            Write-Output ("[{0,3}] slot{1} rc={2} (empty)" -f $p, $s, $rc)
        } else {
            $stat[$s].Rc = $rc
            Write-Output ("[{0,3}] slot{1} rc={2} (0x{2:X8})" -f $p, $s, $rc)
        }
    }
    if ($p -lt $passes -and $IntervalMs -gt 0) { Start-Sleep -Milliseconds $IntervalMs }
}

$live = @()
Write-Output ("---- summary ({0:N1}s) ----" -f ((Get-Date) - $t0).TotalSeconds)
foreach ($s in $slots) {
    $e = $stat[$s]
    if ($e.Live) {
        $live += $s
        $delta = [int64]$e.Last - [int64]$e.First
        Write-Output ("slot{0}: LIVE  pkt {1} -> {2} (delta {3})" -f $s, $e.First, $e.Last, $delta)
    } else {
        Write-Output ("slot{0}: empty (rc={1})" -f $s, $e.Rc)
    }
}
if ($live.Count -gt 0) {
    Write-Output ("VERDICT: XINPUT DEVICE LIVE on slot(s) {0}" -f ($live -join ','))
    exit 0
}
Write-Output "VERDICT: NO XINPUT DEVICE (all slots rc=1167)"
exit 1
