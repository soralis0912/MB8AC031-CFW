/* ifd_api.h -- the IFD handler interface, as far as this driver uses it.
 *
 * The macOS SDK ships the PC/SC client headers (PCSC.framework) but not the
 * driver-side header, so the entry points and the numbers the host side
 * expects are declared here.  They are the published IFD handler v3 ABI,
 * the one pcsc-lite's ifdhandler.h and reader.h define (the names and
 * numbers follow those headers; nothing else is taken from them):
 * values, not code, and every one of them is checked against what
 * /System/Library/CryptoTokenKit/com.apple.ifdreader.slotd calls (its
 * ifdbundle XPC service resolves IFDHCreateChannelByName, IFDHPowerICC,
 * IFDHTransmitToICC, IFDHICCPresence, ... by name).
 *
 * On macOS DWORD is 32 bits (PCSC.framework's wintypes.h) and the return
 * code is a long.  pcsc-lite on Linux makes DWORD an unsigned long (its
 * wintypes.h), so the width follows the platform; the numbers are the same.
 */
#ifndef IFD_API_H
#define IFD_API_H

#include <stdint.h>

#ifdef __APPLE__
typedef uint32_t        DWORD;
#else
typedef unsigned long   DWORD;
#endif
typedef DWORD          *PDWORD;
typedef DWORD          *LPDWORD;
typedef unsigned char   UCHAR;
typedef unsigned char  *PUCHAR;
typedef char           *LPSTR;
typedef long            RESPONSECODE;

typedef struct {
    DWORD Protocol;
    DWORD Length;
} SCARD_IO_HEADER, *PSCARD_IO_HEADER;

/* Return codes. */
#define IFD_SUCCESS                     0
#define IFD_ERROR_TAG                   600
#define IFD_ERROR_SET_FAILURE           601
#define IFD_ERROR_VALUE_READ_ONLY       602
#define IFD_ERROR_PTS_FAILURE           605
#define IFD_ERROR_NOT_SUPPORTED         606
#define IFD_PROTOCOL_NOT_SUPPORTED      607
#define IFD_ERROR_POWER_ACTION          608
#define IFD_COMMUNICATION_ERROR         612
#define IFD_RESPONSE_TIMEOUT            613
#define IFD_NOT_SUPPORTED               614
#define IFD_ICC_PRESENT                 615
#define IFD_ICC_NOT_PRESENT             616
#define IFD_NO_SUCH_DEVICE              617
#define IFD_ERROR_INSUFFICIENT_BUFFER   618

/* IFDHPowerICC actions. */
#define IFD_POWER_UP                    500
#define IFD_POWER_DOWN                  501
#define IFD_RESET                       502

/* IFDHSetProtocolParameters: Protocol is SCARD_PROTOCOL_*, Flags these. */
#define SCARD_PROTOCOL_T0               0x0001
#define SCARD_PROTOCOL_T1               0x0002
#define IFD_NEGOTIATE_PTS1              1
#define IFD_NEGOTIATE_PTS2              2
#define IFD_NEGOTIATE_PTS3              4

/* IFDHGetCapabilities tags. */
#define TAG_IFD_ATR                     0x0303
#define TAG_IFD_SLOTNUM                 0x0180
#define TAG_IFD_SLOT_THREAD_SAFE        0x0FAC
#define TAG_IFD_THREAD_SAFE             0x0FAD
#define TAG_IFD_SLOTS_NUMBER            0x0FAE
#define TAG_IFD_SIMULTANEOUS_ACCESS     0x0FAF
#define TAG_IFD_POLLING_THREAD          0x0FB0
#define SCARD_ATTR_ATR_STRING           0x00090303u
#define SCARD_ATTR_MAXINPUT             0x0007A007u

RESPONSECODE IFDHCreateChannelByName(DWORD Lun, LPSTR DeviceName);
RESPONSECODE IFDHCreateChannel(DWORD Lun, DWORD Channel);
RESPONSECODE IFDHCloseChannel(DWORD Lun);
RESPONSECODE IFDHGetCapabilities(DWORD Lun, DWORD Tag, PDWORD Length,
                                 PUCHAR Value);
RESPONSECODE IFDHSetCapabilities(DWORD Lun, DWORD Tag, DWORD Length,
                                 PUCHAR Value);
RESPONSECODE IFDHSetProtocolParameters(DWORD Lun, DWORD Protocol, UCHAR Flags,
                                       UCHAR PTS1, UCHAR PTS2, UCHAR PTS3);
RESPONSECODE IFDHPowerICC(DWORD Lun, DWORD Action, PUCHAR Atr,
                          PDWORD AtrLength);
RESPONSECODE IFDHTransmitToICC(DWORD Lun, SCARD_IO_HEADER SendPci,
                               PUCHAR TxBuffer, DWORD TxLength,
                               PUCHAR RxBuffer, PDWORD RxLength,
                               PSCARD_IO_HEADER RecvPci);
RESPONSECODE IFDHControl(DWORD Lun, DWORD dwControlCode, PUCHAR TxBuffer,
                         DWORD TxLength, PUCHAR RxBuffer, DWORD RxLength,
                         LPDWORD pdwBytesReturned);
RESPONSECODE IFDHICCPresence(DWORD Lun);

#endif
