#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "sysemu/dma.h"
#include "sysemu/block-backend.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "qemu/iov.h"
#include "hw/scsi/scsi.h"
#include "block/scsi.h"
#include "trace.h"

#include "mptsas.h"
#include "mpi.h"

/**
 * Page address for SAS device page types.
 */
#pragma pack(1)
typedef union MptConfigurationPageAddressSASDevice {
    struct {
        uint16_t    Handle;
        uint16_t    Reserved;
    } Form0And2;
    struct {
        uint8_t     TargetID;
        uint8_t     Bus;
        uint8_t     Reserved;
    } Form1;
} MptConfigurationPageAddressSASDevice;
#pragma pack()

/**
 * Page address for SAS PHY page types.
 */
#pragma pack(1)
typedef union MptConfigurationPageAddressSASPHY {
    struct {
        uint8_t     PhyNumber;
        uint8_t     Reserved[3];
    } Form0;
    struct {
        uint16_t    Index;
        uint16_t    Reserved;
    } Form1;
} MptConfigurationPageAddressSASPHY;
#pragma pack()

/**
 * Union of all possible address types.
 */
#pragma pack(1)
typedef union MptConfigurationPageAddress {
    /** 32bit view. */
    uint32_t PageAddress;
    struct {
        /** Port number to get the configuration page for. */
        uint8_t PortNumber;
        /** Reserved. */
        uint8_t Reserved[3];
    } MPIPortNumber;
    struct {
        /** Target ID to get the configuration page for. */
        uint8_t TargetID;
        /** Bus number to get the configuration page for. */
        uint8_t Bus;
        /** Reserved. */
        uint8_t Reserved[2];
    } BusAndTargetId;
    MptConfigurationPageAddressSASDevice    SASDevice;
    MptConfigurationPageAddressSASPHY       SASPHY;
} MptConfigurationPageAddress;
#pragma pack()

#define MPT_CONFIGURATION_PAGE_ADDRESS_GET_SAS_FORM(x) \
 (((x).PageAddress >> 28) & 0x0f)

/**
 * Configuration Page attributes.
 */
#define MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY            (0x00)
#define MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE          (0x10)
#define MPT_CONFIGURATION_PAGE_ATTRIBUTE_PERSISTENT          (0x20)
#define MPT_CONFIGURATION_PAGE_ATTRIBUTE_PERSISTENT_READONLY (0x30)

#define MPT_CONFIGURATION_PAGE_ATTRIBUTE_GET(PageType) ((PageType) & 0xf0)

/**
 * Configuration Page types.
 */
#define MPT_CONFIGURATION_PAGE_TYPE_IO_UNIT                  (0x00)
#define MPT_CONFIGURATION_PAGE_TYPE_IOC                      (0x01)
#define MPT_CONFIGURATION_PAGE_TYPE_BIOS                     (0x02)
#define MPT_CONFIGURATION_PAGE_TYPE_MANUFACTURING            (0x09)
#define MPT_CONFIGURATION_PAGE_TYPE_EXTENDED                 (0x0F)

#define MPT_CONFIGURATION_PAGE_TYPE_GET(PageType) ((PageType) & 0x0f)

/**
 * Extented page types.
 */
#define MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASIOUNIT       (0x10)
#define MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASEXPANDER     (0x11)
#define MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASDEVICE       (0x12)
#define MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASPHYS         (0x13)
#define MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_LOG             (0x14)
#define MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_ENCLOSURE       (0x15)

/**
 * Configuration Page header - Common to all pages.
 */
#pragma pack(1)
typedef struct MptConfigurationPageHeader {
    /** Version of the page. */
    uint8_t     PageVersion;
    /** The length of the page in 32bit D-Words. */
    uint8_t     PageLength;
    /** Number of the page. */
    uint8_t     PageNumber;
    /** Type of the page. */
    uint8_t     PageType;
} MptConfigurationPageHeader;
#pragma pack()

/**
 * Extended configuration page header - Common to all extended pages.
 */
#pragma pack(1)
typedef struct MptExtendedConfigurationPageHeader {
    /** Version of the page. */
    uint8_t     PageVersion;
    /** Reserved. */
    uint8_t     Reserved1;
    /** Number of the page. */
    uint8_t     PageNumber;
    /** Type of the page. */
    uint8_t     PageType;
    /** Extended page length. */
    uint16_t    ExtPageLength;
    /** Extended page type. */
    uint8_t     ExtPageType;
    /** Reserved */
    uint8_t     Reserved2;
} MptExtendedConfigurationPageHeader;
#pragma pack()

/**
 * Manufacturing page 0. - Readonly.
 */
#pragma pack(1)
typedef struct MptConfigurationPageManufacturing0 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                   abPageData[76];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** Name of the chip. */
            uint8_t               abChipName[16];
            /** Chip revision. */
            uint8_t               abChipRevision[8];
            /** Board name. */
            uint8_t               abBoardName[16];
            /** Board assembly. */
            uint8_t               abBoardAssembly[16];
            /** Board tracer number. */
            uint8_t               abBoardTracerNumber[16];
        } fields;
    } u;
} MptConfigurationPageManufacturing0;
#pragma pack()

/**
 * Manufacturing page 1. - Readonly Persistent.
 */
#pragma pack(1)
typedef struct MptConfigurationPageManufacturing1 {
    /** Union */
    union {
        /** Byte view */
        uint8_t                           abPageData[260];
        /** Field view */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** VPD info - don't know what belongs here so all zero. */
            uint8_t                       abVPDInfo[256];
        } fields;
    } u;
} MptConfigurationPageManufacturing1;
#pragma pack()

/**
 * Manufacturing page 2. - Readonly.
 */
#pragma pack(1)
typedef struct MptConfigurationPageManufacturing2 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                        abPageData[8];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader Header;
            /** PCI Device ID. */
            uint16_t                   PCIDeviceID;
            /** PCI Revision ID. */
            uint8_t                    PCIRevisionID;
            /** Reserved. */
            uint8_t                    Reserved;
            /** Hardware specific settings... */
        } fields;
    } u;
} MptConfigurationPageManufacturing2;
#pragma pack()

/**
 * Manufacturing page 3. - Readonly.
 */
#pragma pack(1)
typedef struct MptConfigurationPageManufacturing3 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                   abPageData[8];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** PCI Device ID. */
            uint16_t              PCIDeviceID;
            /** PCI Revision ID. */
            uint8_t               PCIRevisionID;
            /** Reserved. */
            uint8_t               Reserved;
            /** Chip specific settings... */
        } fields;
    } u;
} MptConfigurationPageManufacturing3;
#pragma pack()

/**
 * Manufacturing page 4. - Readonly.
 */
#pragma pack(1)
typedef struct MptConfigurationPageManufacturing4 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                   abPageData[84];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** Reserved. */
            uint32_t              Reserved;
            /** InfoOffset0. */
            uint8_t               InfoOffset0;
            /** Info size. */
            uint8_t               InfoSize0;
            /** InfoOffset1. */
            uint8_t               InfoOffset1;
            /** Info size. */
            uint8_t               InfoSize1;
            /** Size of the inquiry data. */
            uint8_t               InquirySize;
            /** Reserved. */
            uint8_t               abReserved[3];
            /** Inquiry data. */
            uint8_t               abInquiryData[56];
            /** IS volume settings. */
            uint32_t              ISVolumeSettings;
            /** IME volume settings. */
            uint32_t              IMEVolumeSettings;
            /** IM volume settings. */
            uint32_t              IMVolumeSettings;
        } fields;
    } u;
} MptConfigurationPageManufacturing4;
#pragma pack()

/**
 * Manufacturing page 5 - Readonly.
 */
#pragma pack(1)
typedef struct MptConfigurationPageManufacturing5 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                           abPageData[88];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** Base WWID. */
            uint64_t                      BaseWWID;
            /** Flags */
            uint8_t                       Flags;
            /** Number of ForceWWID fields in this page. */
            uint8_t                       NumForceWWID;
            /** Reserved */
            uint16_t                      Reserved;
            /** Reserved */
            uint32_t                      au32Reserved[2];
            /** ForceWWID entries  Maximum of 8 because the SAS
               controller doesn't has more */
            uint64_t                      au64ForceWWID[8];
        } fields;
    } u;
} MptConfigurationPageManufacturing5;
#pragma pack()

/**
 * Manufacturing page 6 - Readonly.
 */
#pragma pack(1)
typedef struct MptConfigurationPageManufacturing6 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                           abPageData[4];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** Product specific data - 0 for now */
        } fields;
    } u;
} MptConfigurationPageManufacturing6;
#pragma pack()

/**
 * Manufacutring page 7 - PHY element.
 */
#pragma pack(1)
typedef struct MptConfigurationPageManufacturing7PHY {
    /** Pinout */
    uint32_t                  Pinout;
    /** Connector name */
    uint8_t                   szConnector[16];
    /** Location */
    uint8_t                   Location;
    /** reserved */
    uint8_t                   Reserved;
    /** Slot */
    uint16_t                  Slot;
} MptConfigurationPageManufacturing7PHY;
#pragma pack()

/**
 * Manufacturing page 7 - Readonly.
 */
#pragma pack(1)
typedef struct MptConfigurationPageManufacturing7 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                           abPageData[1];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** Reserved */
            uint32_t                      au32Reserved[2];
            /** Flags */
            uint32_t                      Flags;
            /** Enclosure name */
            uint8_t                       szEnclosureName[16];
            /** Number of PHYs */
            uint8_t                       NumPhys;
            /** Reserved */
            uint8_t                       au8Reserved[3];
            /** PHY list for the SAS controller -
                variable depending on the number of ports */
            MptConfigurationPageManufacturing7PHY aPHY[MPTSAS_NUM_PORTS];
        } fields;
    } u;
} MptConfigurationPageManufacturing7;
#pragma pack()

/** Flags for the flags field */
#define MPTSAS_MANUFACTURING7_FLAGS_USE_PROVIDED_INFORMATION (1<<0)

/** Flags for the pinout field */
#define MPTSAS_MANUFACTURING7_PINOUT_UNKNOWN                 (1<<0)
#define MPTSAS_MANUFACTURING7_PINOUT_SFF8482                 (1<<1)
#define MPTSAS_MANUFACTURING7_PINOUT_SFF8470_LANE1           (1<<8)
#define MPTSAS_MANUFACTURING7_PINOUT_SFF8470_LANE2           (1<<9)
#define MPTSAS_MANUFACTURING7_PINOUT_SFF8470_LANE3           (1<<10)
#define MPTSAS_MANUFACTURING7_PINOUT_SFF8470_LANE4           (1<<11)
#define MPTSAS_MANUFACTURING7_PINOUT_SFF8484_LANE1           (1<<16)
#define MPTSAS_MANUFACTURING7_PINOUT_SFF8484_LANE2           (1<<17)
#define MPTSAS_MANUFACTURING7_PINOUT_SFF8484_LANE3           (1<<18)
#define MPTSAS_MANUFACTURING7_PINOUT_SFF8484_LANE4           (1<<19)

/** Flags for the location field */
#define MPTSAS_MANUFACTURING7_LOCATION_UNKNOWN               0x01
#define MPTSAS_MANUFACTURING7_LOCATION_INTERNAL              0x02
#define MPTSAS_MANUFACTURING7_LOCATION_EXTERNAL              0x04
#define MPTSAS_MANUFACTURING7_LOCATION_SWITCHABLE            0x08
#define MPTSAS_MANUFACTURING7_LOCATION_AUTO                  0x10
#define MPTSAS_MANUFACTURING7_LOCATION_NOT_PRESENT           0x20
#define MPTSAS_MANUFACTURING7_LOCATION_NOT_CONNECTED         0x80

/**
 * Manufacturing page 8 - Readonly.
 */
#pragma pack(1)
typedef struct MptConfigurationPageManufacturing8 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                           abPageData[4];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** Product specific information */
        } fields;
    } u;
} MptConfigurationPageManufacturing8;
#pragma pack()

/**
 * Manufacturing page 9 - Readonly.
 */
#pragma pack(1)
typedef struct MptConfigurationPageManufacturing9 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                           abPageData[4];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** Product specific information */
        } fields;
    } u;
} MptConfigurationPageManufacturing9;
#pragma pack()

/**
 * Manufacturing page 10 - Readonly.
 */
#pragma pack(1)
typedef struct MptConfigurationPageManufacturing10 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                           abPageData[4];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** Product specific information */
        } fields;
    } u;
} MptConfigurationPageManufacturing10;
#pragma pack()

/**
 * IO Unit page 0. - Readonly.
 */
#pragma pack(1)
typedef struct MptConfigurationPageIOUnit0 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                   abPageData[12];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** A unique identifier. */
            uint64_t              UniqueIdentifier;
        } fields;
    } u;
} MptConfigurationPageIOUnit0;
#pragma pack()

/**
 * IO Unit page 1. - Read/Write.
 */
#pragma pack(1)
typedef struct MptConfigurationPageIOUnit1 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                   abPageData[8];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** Flag whether this is a single function PCI device. */
            unsigned              SingleFunction:1;
            /** Flag whether all possible paths to a device are mapped. */
            unsigned              AllPathsMapped:1;
            /** Reserved. */
            unsigned              Reserved:4;
            /** Flag whether all RAID functionality is disabled. */
            unsigned              IntegratedRAIDDisabled:1;
            /** Flag whether 32bit PCI accesses are forced. */
            unsigned              BitAccessForced:1;
            /** Reserved. */
            unsigned              abReserved:24;
        } fields;
    } u;
} MptConfigurationPageIOUnit1;
#pragma pack()

/**
 * Adapter Ordering.
 */
#pragma pack(1)
typedef struct MptConfigurationPageIOUnit2AdapterOrdering {
    /** PCI bus number. */
    unsigned    PCIBusNumber:8;
    /** PCI device and function number. */
    unsigned    PCIDevFn:8;
    /** Flag whether the adapter is embedded. */
    unsigned    AdapterEmbedded:1;
    /** Flag whether the adapter is enabled. */
    unsigned    AdapterEnabled:1;
    /** Reserved. */
    unsigned    :6;
    /** Reserved. */
    unsigned    :8;
} MptConfigurationPageIOUnit2AdapterOrdering;
#pragma pack()

/**
 * IO Unit page 2. - Read/Write.
 */
#pragma pack(1)
typedef struct MptConfigurationPageIOUnit2 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                   abPageData[28];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** Reserved. */
            unsigned              :1;
            /** Flag whether Pause on error is enabled. */
            unsigned              PauseOnError:1;
            /** Flag whether verbose mode is enabled. */
            unsigned              VerboseModeEnabled:1;
            /** Set to disable color video. */
            unsigned              DisableColorVideo:1;
            /** Flag whether int 40h is hooked. */
            unsigned              NotHookInt40h:1;
            /** Reserved. */
            unsigned              :3;
            /** Reserved. */
            unsigned              :24;
            /** BIOS version. */
            uint32_t              BIOSVersion;
            /** Adapter ordering. */
            MptConfigurationPageIOUnit2AdapterOrdering aAdapterOrder[4];
        } fields;
    } u;
} MptConfigurationPageIOUnit2;
#pragma pack()

/*
 * IO Unit page 3. - Read/Write.
 */
#pragma pack(1)
typedef struct MptConfigurationPageIOUnit3 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                   abPageData[8];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** Number of GPIO values. */
            uint8_t               GPIOCount;
            /** Reserved. */
            uint8_t               abReserved[3];
        } fields;
    } u;
} MptConfigurationPageIOUnit3;
#pragma pack()

/*
 * IO Unit page 4. - Readonly for everyone except the BIOS.
 */
#pragma pack(1)
typedef struct MptConfigurationPageIOUnit4 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                   abPageData[20];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** Reserved */
            uint32_t                      Reserved;
            /** SG entry describing the Firmware location. */
            uint32_t                      FlagsLength;
            uint64_t                      Address64;
        } fields;
    } u;
} MptConfigurationPageIOUnit4;
#pragma pack()

/**
 * IOC page 0. - Readonly
 */
#pragma pack(1)
typedef struct MptConfigurationPageIOC0 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                   abPageData[28];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** Total amount of NV memory in bytes. */
            uint32_t              TotalNVStore;
            /** Number of free bytes in the NV store. */
            uint32_t              FreeNVStore;
            /** PCI vendor ID. */
            uint16_t              VendorId;
            /** PCI device ID. */
            uint16_t              DeviceId;
            /** PCI revision ID. */
            uint8_t               RevisionId;
            /** Reserved. */
            uint8_t               abReserved[3];
            /** PCI class code. */
            uint32_t              ClassCode;
            /** Subsystem vendor Id. */
            uint16_t              SubsystemVendorId;
            /** Subsystem Id. */
            uint16_t              SubsystemId;
        } fields;
    } u;
} MptConfigurationPageIOC0;
#pragma pack()

/**
 * IOC page 1. - Read/Write
 */
#pragma pack(1)
typedef struct MptConfigurationPageIOC1 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                   abPageData[16];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** Flag whether reply coalescing is enabled. */
            unsigned              ReplyCoalescingEnabled:1;
            /** Reserved. */
            unsigned              Reserved:31;
            /** Coalescing Timeout in microseconds. */
            unsigned              CoalescingTimeout:32;
            /** Coalescing depth. */
            unsigned              CoalescingDepth:8;
            /** Reserved. */
            unsigned              Reserved0:8;
            unsigned              Reserved1:8;
            unsigned              Reserved2:8;
        } fields;
    } u;
} MptConfigurationPageIOC1;
#pragma pack()

/**
 * IOC page 2. - Readonly
 */
#pragma pack(1)
typedef struct MptConfigurationPageIOC2 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                   abPageData[12];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** Flag whether striping is supported. */
            unsigned              StripingSupported:1;
            /** Flag whether enhanced mirroring is supported. */
            unsigned              EnhancedMirroringSupported:1;
            /** Flag whether mirroring is supported. */
            unsigned              MirroringSupported:1;
            /** Reserved. */
            unsigned              Reserved:26;
            /** Flag whether SES is supported. */
            unsigned              SESSupported:1;
            /** Flag whether SAF-TE is supported. */
            unsigned              SAFTESupported:1;
            /** Flag whether cross channel volumes are supported. */
            unsigned              CrossChannelVolumesSupported:1;
            /** Number of active integrated RAID volumes. */
            unsigned              NumActiveVolumes:8;
            /** Maximum number of integrated RAID volumes supported. */
            unsigned              MaxVolumes:8;
            /** Number of active integrated RAID physical disks. */
            unsigned              NumActivePhysDisks:8;
            /** Maximum number of integrated RAID physical disks supported. */
            unsigned              MaxPhysDisks:8;
            /** RAID volumes... - not supported. */
        } fields;
    } u;
} MptConfigurationPageIOC2;
#pragma pack()

/**
 * IOC page 3. - Readonly
 */
#pragma pack(1)
typedef struct MptConfigurationPageIOC3 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                   abPageData[8];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** Number of active integrated RAID physical disks. */
            uint8_t               NumPhysDisks;
            /** Reserved. */
            uint8_t               abReserved[3];
        } fields;
    } u;
} MptConfigurationPageIOC3;
#pragma pack()

/**
 * IOC page 4. - Read/Write
 */
#pragma pack(1)
typedef struct MptConfigurationPageIOC4 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                   abPageData[8];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** Number of SEP entries in this page. */
            uint8_t               ActiveSEP;
            /** Maximum number of SEp entries supported. */
            uint8_t               MaxSEP;
            /** Reserved. */
            uint16_t              Reserved;
            /** SEP entries... - not supported. */
        } fields;
    } u;
} MptConfigurationPageIOC4;
#pragma pack()

/**
 * IOC page 6. - Read/Write
 */
#pragma pack(1)
typedef struct MptConfigurationPageIOC6 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                   abPageData[60];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            uint32_t                      CapabilitiesFlags;
            uint8_t                       MaxDrivesIS;
            uint8_t                       MaxDrivesIM;
            uint8_t                       MaxDrivesIME;
            uint8_t                       Reserved1;
            uint8_t                       MinDrivesIS;
            uint8_t                       MinDrivesIM;
            uint8_t                       MinDrivesIME;
            uint8_t                       Reserved2;
            uint8_t                       MaxGlobalHotSpares;
            uint8_t                       Reserved3;
            uint16_t                      Reserved4;
            uint32_t                      Reserved5;
            uint32_t                      SupportedStripeSizeMapIS;
            uint32_t                      SupportedStripeSizeMapIME;
            uint32_t                      Reserved6;
            uint8_t                       MetadataSize;
            uint8_t                       Reserved7;
            uint16_t                      Reserved8;
            uint16_t                      MaxBadBlockTableEntries;
            uint16_t                      Reserved9;
            uint16_t                      IRNvsramUsage;
            uint16_t                      Reserved10;
            uint32_t                      IRNvsramVersion;
            uint32_t                      Reserved11;
        } fields;
    } u;
} MptConfigurationPageIOC6;
#pragma pack()

/**
 * BIOS page 1 - Read/write.
 */
#pragma pack(1)
typedef struct MptConfigurationPageBIOS1 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                   abPageData[48];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** BIOS options */
            uint32_t                      BiosOptions;
            /** IOC settings */
            uint32_t                      IOCSettings;
            /** Reserved */
            uint32_t                      Reserved1;
            /** Device settings */
            uint32_t                      DeviceSettings;
            /** Number of devices */
            uint16_t                      NumberOfDevices;
            /** Expander spinup */
            uint8_t                       ExpanderSpinup;
            /** Reserved */
            uint8_t                       Reserved2;
            /** I/O timeout of block devices without removable media */
            uint16_t                      IOTimeoutBlockDevicesNonRM;
            /** I/O timeout sequential */
            uint16_t                      IOTimeoutSequential;
            /** I/O timeout other */
            uint16_t                      IOTimeoutOther;
            /** I/O timeout of block devices with removable media */
            uint16_t                      IOTimeoutBlockDevicesRM;
        } fields;
    } u;
} MptConfigurationPageBIOS1;
#pragma pack()

#define MPTSAS_BIOS1_BIOSOPTIONS_BIOS_DISABLE              (1<<0)
#define MPTSAS_BIOS1_BIOSOPTIONS_SCAN_FROM_HIGH_TO_LOW     (1<<1)
#define MPTSAS_BIOS1_BIOSOPTIONS_BIOS_EXTENDED_SAS_SUPPORT (1<<8)
#define MPTSAS_BIOS1_BIOSOPTIONS_BIOS_EXTENDED_FC_SUPPORT  (1<<9)
#define MPTSAS_BIOS1_BIOSOPTIONS_BIOS_EXTENDED_SPI_SUPPORT (1<<10)

#define MPTSAS_BIOS1_IOCSETTINGS_ALTERNATE_CHS             (1<<3)

#define MPTSAS_BIOS1_IOCSETTINGS_ADAPTER_SUPPORT_SET(x)    ((x) << 4)
#define MPTSAS_BIOS1_IOCSETTINGS_ADAPTER_SUPPORT_DISABLED  0x00
#define MPTSAS_BIOS1_IOCSETTINGS_ADAPTER_SUPPORT_BIOS_ONLY 0x01
#define MPTSAS_BIOS1_IOCSETTINGS_ADAPTER_SUPPORT_OS_ONLY   0x02
#define MPTSAS_BIOS1_IOCSETTINGS_ADAPTER_SUPPORT_BOT       0x03

#define MPTSAS_BIOS1_IOCSETTINGS_REMOVABLE_MEDIA_SET(x)    ((x) << 6)
#define MPTSAS_BIOS1_IOCSETTINGS_REMOVABLE_MEDIA_NO_INT13H 0x00
#define MPTSAS_BIOS1_IOCSETTINGS_REMOVABLE_BOOT_MEDIA_INT13H 0x01
#define MPTSAS_BIOS1_IOCSETTINGS_REMOVABLE_MEDIA_INT13H      0x02

#define MPTSAS_BIOS1_IOCSETTINGS_SPINUP_DELAY_SET(x) \
 ((x & 0xF) << 8)
#define MPTSAS_BIOS1_IOCSETTINGS_SPINUP_DELAY_GET(x) \
 ((x >> 8) & 0x0F)
#define MPTSAS_BIOS1_IOCSETTINGS_MAX_TARGET_SPINUP_SET(x) \
 ((x & 0xF) << 12)
#define MPTSAS_BIOS1_IOCSETTINGS_MAX_TARGET_SPINUP_GET(x) \
 ((x >> 12) & 0x0F)

#define MPTSAS_BIOS1_IOCSETTINGS_BOOT_PREFERENCE_SET(x) \
 (((x) & 0x3) << 16)
#define MPTSAS_BIOS1_IOCSETTINGS_BOOT_PREFERENCE_ENCLOSURE   0x0
#define MPTSAS_BIOS1_IOCSETTINGS_BOOT_PREFERENCE_SAS_ADDRESS 0x1

#define MPTSAS_BIOS1_IOCSETTINGS_DIRECT_ATTACH_SPINUP_MODE_ALL (1<<18)
#define MPTSAS_BIOS1_IOCSETTINGS_AUTO_PORT_ENABLE              (1<<19)

#define MPTSAS_BIOS1_IOCSETTINGS_PORT_ENABLE_REPLY_DELAY_SET(x) \
 (((x) & 0xF) << 20)
#define MPTSAS_BIOS1_IOCSETTINGS_PORT_ENABLE_REPLY_DELAY_GET(x) \
 ((x >> 20) & 0x0F)

#define MPTSAS_BIOS1_IOCSETTINGS_PORT_ENABLE_SPINUP_DELAY_SET(x) \
 (((x) & 0xF) << 24)
#define MPTSAS_BIOS1_IOCSETTINGS_PORT_ENABLE_SPINUP_DELAY_GET(x) \
 ((x >> 24) & 0x0F)

#define MPTSAS_BIOS1_DEVSETTINGS_DISABLE_LUN_SCANS      (1<<0)
#define MPTSAS_BIOS1_DEVSETTINGS_DISABLE_LUN_SCANS_NON_REMOVABLE_DEVS \
 (1<<1)
#define MPTSAS_BIOS1_DEVSETTINGS_DISABLE_LUN_SCANS_REMOVABLE_DEVS (1<<2)
#define MPTSAS_BIOS1_DEVSETTINGS_DISABLE_LUN_SCANS2     (1<<3)
#define MPTSAS_BIOS1_DEVSETTINGS_DISABLE_SMART_POLLING  (1<<4)

#define MPTSAS_BIOS1_EXPANDERSPINUP_SPINUP_DELAY_SET(x) ((x) & 0x0F)
#define MPTSAS_BIOS1_EXPANDERSPINUP_SPINUP_DELAY_GET(x) ((x) & 0x0F)
#define MPTSAS_BIOS1_EXPANDERSPINUP_MAX_SPINUP_DELAY_SET(x) \
 (((x) & 0x0F) << 4)
#define MPTSAS_BIOS1_EXPANDERSPINUP_MAX_SPINUP_DELAY_GET(x) \
 ((x >> 4) & 0x0F)

/**
 * BIOS page 2 - Read/write.
 */
#pragma pack(1)
typedef struct MptConfigurationPageBIOS2 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                   abPageData[384];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** Reserved */
            uint32_t                      au32Reserved[6];
            /** Format of the boot device field. */
            uint8_t                       BootDeviceForm;
            /** Previous format of the boot device field. */
            uint8_t                       PrevBootDeviceForm;
            /** Reserved */
            uint16_t                      Reserved;
            /** Boot device fields - dependent on the format */
            uint32_t                      au32Reserved2[64];
        } fields;
    } u;
} MptConfigurationPageBIOS2;
#pragma pack()

#define MPTSAS_BIOS2_BOOT_DEVICE_FORM_SET(x)                 ((x) & 0x0F)
#define MPTSAS_BIOS2_BOOT_DEVICE_FORM_FIRST                  0x0
#define MPTSAS_BIOS2_BOOT_DEVICE_FORM_ADAPTER_BUS_TARGET_LUN 0x1
#define MPTSAS_BIOS2_BOOT_DEVICE_FORM_PCIADDR_BUS_TARGET_LUN 0x2
#define MPTSAS_BIOS2_BOOT_DEVICE_FORM_PCISLOT_BUS_TARGET_LUN 0x3
#define MPTSAS_BIOS2_BOOT_DEVICE_FORM_FC_WWN                 0x4
#define MPTSAS_BIOS2_BOOT_DEVICE_FORM_SAS_WWN                0x5
#define MPTSAS_BIOS2_BOOT_DEVICE_FORM_ENCLOSURE_SLOT         0x6

/**
 * BIOS page 4 - Read/Write (Where is 3? - not defined in the spec)
 */
#pragma pack(1)
typedef struct MptConfigurationPageBIOS4 {
    /** Union. */
    union {
        /** Byte view. */
        uint8_t                   abPageData[12];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptConfigurationPageHeader    Header;
            /** Reassignment Base WWID */
            uint64_t                      ReassignmentBaseWWID;
        } fields;
    } u;
} MptConfigurationPageBIOS4;
#pragma pack()

/**
 * Device settings for one device.
 */
#pragma pack(1)
typedef struct MptDeviceSettings {
    /** Timeout for I/O in seconds. */
    unsigned    Timeout:8;
    /** Minimum synchronous factor. */
    unsigned    SyncFactor:8;
    /** Flag whether disconnect is enabled. */
    unsigned    DisconnectEnable:1;
    /** Flag whether Scan ID is enabled. */
    unsigned    ScanIDEnable:1;
    /** Flag whether Scan LUNs is enabled. */
    unsigned    ScanLUNEnable:1;
    /** Flag whether tagged queuing is enabled. */
    unsigned    TaggedQueuingEnabled:1;
    /** Flag whether wide is enabled. */
    unsigned    WideDisable:1;
    /** Flag whether this device is bootable. */
    unsigned    BootChoice:1;
    /** Reserved. */
    unsigned    Reserved:10;
} MptDeviceSettings;
#pragma pack()

/**
 * PHY entry for the SAS I/O unit page 0
 */
#pragma pack(1)
typedef struct MptConfigurationPageSASIOUnit0PHY {
    /** Port number */
    uint8_t                           Port;
    /** Port flags */
    uint8_t                           PortFlags;
    /** Phy flags */
    uint8_t                           PhyFlags;
    /** negotiated link rate */
    uint8_t                           NegotiatedLinkRate;
    /** Controller phy device info */
    uint32_t                          ControllerPhyDeviceInfo;
    /** Attached device handle */
    uint16_t                          AttachedDevHandle;
    /** Controller device handle */
    uint16_t                          ControllerDevHandle;
    /** Discovery status */
    uint32_t                          DiscoveryStatus;
} MptConfigurationPageSASIOUnit0PHY;
#pragma pack()

/**
 * SAS I/O  Unit page 0 - Readonly
 */
#pragma pack(1)
typedef struct MptConfigurationPageSASIOUnit0 {
    /** Union. */
    union {
        /** Byte view - variable. */
        uint8_t                                   abPageData[1];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptExtendedConfigurationPageHeader    ExtHeader;
            /** Nvdata version default */
            uint16_t                              NvdataVersionDefault;
            /** Nvdata version persistent */
            uint16_t                              NvdataVersionPersistent;
            /** Number of physical ports */
            uint8_t                               NumPhys;
            /** Reserved */
            uint8_t                               au8Reserved[3];
            /** Content for each physical port -
                variable depending on the amount of ports. */
            MptConfigurationPageSASIOUnit0PHY     aPHY[MPTSAS_NUM_PORTS];
        } fields;
    } u;
} MptConfigurationPageSASIOUnit0;
#pragma pack()

#define MPTSAS_SASIOUNIT0_PORT_CONFIGURATION_AUTO  (1<<0)
#define MPTSAS_SASIOUNIT0_PORT_TARGET_IOC          (1<<2)
#define MPTSAS_SASIOUNIT0_PORT_DISCOVERY_IN_STATUS (1<<3)

#define MPTSAS_SASIOUNIT0_PHY_RX_INVERTED          (1<<0)
#define MPTSAS_SASIOUNIT0_PHY_TX_INVERTED          (1<<1)
#define MPTSAS_SASIOUNIT0_PHY_DISABLED             (1<<2)

#define MPTSAS_SASIOUNIT0_NEGOTIATED_RATE_SET(x)   ((x) & 0x0F)
#define MPTSAS_SASIOUNIT0_NEGOTIATED_RATE_GET(x)   ((x) & 0x0F)
#define MPTSAS_SASIOUNIT0_NEGOTIATED_RATE_UNKNOWN  0x00
#define MPTSAS_SASIOUNIT0_NEGOTIATED_RATE_DISABLED 0x01
#define MPTSAS_SASIOUNIT0_NEGOTIATED_RATE_FAILED   0x02
#define MPTSAS_SASIOUNIT0_NEGOTIATED_RATE_SATA_OOB 0x03
#define MPTSAS_SASIOUNIT0_NEGOTIATED_RATE_15GB     0x08
#define MPTSAS_SASIOUNIT0_NEGOTIATED_RATE_30GB     0x09

#define MPTSAS_SASIOUNIT0_DEVICE_TYPE_SET(x)          ((x) & 0x3)
#define MPTSAS_SASIOUNIT0_DEVICE_TYPE_NO              0x0
#define MPTSAS_SASIOUNIT0_DEVICE_TYPE_END             0x1
#define MPTSAS_SASIOUNIT0_DEVICE_TYPE_EDGE_EXPANDER   0x2
#define MPTSAS_SASIOUNIT0_DEVICE_TYPE_FANOUT_EXPANDER 0x3

#define MPTSAS_SASIOUNIT0_DEVICE_SATA_HOST            (1<<3)
#define MPTSAS_SASIOUNIT0_DEVICE_SMP_INITIATOR        (1<<4)
#define MPTSAS_SASIOUNIT0_DEVICE_STP_INITIATOR        (1<<5)
#define MPTSAS_SASIOUNIT0_DEVICE_SSP_INITIATOR        (1<<6)
#define MPTSAS_SASIOUNIT0_DEVICE_SATA                 (1<<7)
#define MPTSAS_SASIOUNIT0_DEVICE_SMP_TARGET           (1<<8)
#define MPTSAS_SASIOUNIT0_DEVICE_STP_TARGET           (1<<9)
#define MPTSAS_SASIOUNIT0_DEVICE_SSP_TARGET           (1<<10)
#define MPTSAS_SASIOUNIT0_DEVICE_DIRECT_ATTACHED      (1<<11)
#define MPTSAS_SASIOUNIT0_DEVICE_LSI                  (1<<12)
#define MPTSAS_SASIOUNIT0_DEVICE_ATAPI_DEVICE         (1<<13)
#define MPTSAS_SASIOUNIT0_DEVICE_SEP_DEVICE           (1<<14)

#define MPTSAS_SASIOUNIT0_DISCOVERY_STATUS_LOOP            (1<<0)
#define MPTSAS_SASIOUNIT0_DISCOVERY_STATUS_UNADDRESSABLE   (1<<1)
#define MPTSAS_SASIOUNIT0_DISCOVERY_STATUS_SAME_SAS_ADDR   (1<<2)
#define MPTSAS_SASIOUNIT0_DISCOVERY_STATUS_EXPANDER_ERROR  (1<<3)
#define MPTSAS_SASIOUNIT0_DISCOVERY_STATUS_SMP_TIMEOUT     (1<<4)
#define MPTSAS_SASIOUNIT0_DISCOVERY_STATUS_EXP_ROUTE_OOE   (1<<5)
#define MPTSAS_SASIOUNIT0_DISCOVERY_STATUS_EXP_ROUTE_IDX   (1<<6)
#define MPTSAS_SASIOUNIT0_DISCOVERY_STATUS_SMP_FUNC_FAILED (1<<7)
#define MPTSAS_SASIOUNIT0_DISCOVERY_STATUS_SMP_CRC_ERROR   (1<<8)
#define MPTSAS_SASIOUNIT0_DISCOVERY_STATUS_SUBTRSCTIVE_LNK (1<<9)
#define MPTSAS_SASIOUNIT0_DISCOVERY_STATUS_TBL_LNK         (1<<10)
#define MPTSAS_SASIOUNIT0_DISCOVERY_STATUS_UNSUPPORTED_DEV (1<<11)
#define MPTSAS_SASIOUNIT0_DISCOVERY_STATUS_MAX_SATA_TGTS   (1<<12)
#define MPTSAS_SASIOUNIT0_DISCOVERY_STATUS_MULT_CTRLS      (1<<13)

/**
 * PHY entry for the SAS I/O unit page 1
 */
#pragma pack(1)
typedef struct MptConfigurationPageSASIOUnit1PHY {
    /** Port number */
    uint8_t                           Port;
    /** Port flags */
    uint8_t                           PortFlags;
    /** Phy flags */
    uint8_t                           PhyFlags;
    /** Max link rate */
    uint8_t                           MaxMinLinkRate;
    /** Controller phy device info */
    uint32_t                          ControllerPhyDeviceInfo;
    /** Maximum target port connect time */
    uint16_t                          MaxTargetPortConnectTime;
    /** Reserved */
    uint16_t                          Reserved;
} MptConfigurationPageSASIOUnit1PHY;
#pragma pack()

/**
 * SAS I/O  Unit page 1 - Read/Write
 */
#pragma pack(1)
typedef struct MptConfigurationPageSASIOUnit1 {
    /** Union. */
    union {
        /** Byte view - variable. */
        uint8_t                                   abPageData[1];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptExtendedConfigurationPageHeader    ExtHeader;
            /** Control flags */
            uint16_t                              ControlFlags;
            /** maximum number of SATA targets */
            uint16_t                              MaxNumSATATargets;
            /** additional control flags */
            uint16_t                              AdditionalControlFlags;
            /** Reserved */
            uint16_t                              Reserved;
            /** Number of PHYs */
            uint8_t                               NumPhys;
            /** maximum SATA queue depth */
            uint8_t                               SATAMaxQDepth;
            /** Delay for reporting missing devices. */
            uint8_t                               ReportDeviceMissingDelay;
            /** I/O device missing delay */
            uint8_t                               IODeviceMissingDelay;
            /** Content for each physical port -
                variable depending on the number of ports */
            MptConfigurationPageSASIOUnit1PHY     aPHY[MPTSAS_NUM_PORTS];
        } fields;
    } u;
} MptConfigurationPageSASIOUnit1;
#pragma pack()

#define MPTSAS_SASIOUNIT1_CONTROL_CLEAR_SATA_AFFILIATION     (1<<0)
#define MPTSAS_SASIOUNIT1_CONTROL_FIRST_LEVEL_DISCOVERY_ONLY (1<<1)
#define MPTSAS_SASIOUNIT1_CONTROL_SUBTRACTIVE_LNK_ILLEGAL    (1<<2)
#define MPTSAS_SASIOUNIT1_CONTROL_IOC_ENABLE_HIGH_PHY        (1<<3)
#define MPTSAS_SASIOUNIT1_CONTROL_SATA_FUA_REQUIRED          (1<<4)
#define MPTSAS_SASIOUNIT1_CONTROL_SATA_NCQ_REQUIRED          (1<<5)
#define MPTSAS_SASIOUNIT1_CONTROL_SATA_SMART_REQUIRED        (1<<6)
#define MPTSAS_SASIOUNIT1_CONTROL_SATA_LBA48_REQUIRED        (1<<7)
#define MPTSAS_SASIOUNIT1_CONTROL_SATA_INIT_POSTPONED        (1<<8)

#define MPTSAS_SASIOUNIT1_CONTROL_DEVICE_SUPPORT_SET(x) \
 (((x) & 0x3) << 9)
#define MPTSAS_SASIOUNIT1_CONTROL_DEVICE_SUPPORT_GET(x) \
 (((x) >> 9) & 0x3)
#define MPTSAS_SASIOUNIT1_CONTROL_DEVICE_SUPPORT_SAS_AND_SATA 0x00
#define MPTSAS_SASIOUNIT1_CONTROL_DEVICE_SUPPORT_SAS          0x01
#define MPTSAS_SASIOUNIT1_CONTROL_DEVICE_SUPPORT_SATA         0x02

#define MPTSAS_SASIOUNIT1_CONTROL_SATA_EXP_ADDR                  (1<<11)
#define MPTSAS_SASIOUNIT1_CONTROL_SATA_SETTINGS_PRESERV_REQUIRED (1<<12)
#define MPTSAS_SASIOUNIT1_CONTROL_SATA_LIMIT_RATE_15GB           (1<<13)
#define MPTSAS_SASIOUNIT1_CONTROL_SATA_LIMIT_RATE_30GB           (1<<14)
#define MPTSAS_SASIOUNIT1_CONTROL_SAS_SELF_TEST_ENABLED          (1<<15)

#define MPTSAS_SASIOUNIT1_ADDITIONAL_CONTROL_TBL_LNKS_ALLOW        (1<<0)
#define MPTSAS_SASIOUNIT1_ADDITIONAL_CONTROL_SATA_RST_NO_AFFIL     (1<<1)
#define MPTSAS_SASIOUNIT1_ADDITIONAL_CONTROL_SATA_RST_SELF_AFFIL   (1<<2)
#define MPTSAS_SASIOUNIT1_ADDITIONAL_CONTROL_SATA_RST_OTHER_AFFIL  (1<<3)
#define MPTSAS_SASIOUNIT1_ADDITIONAL_CONTROL_SATA_RST_PORT_EN_ONLY (1<<4)
#define MPTSAS_SASIOUNIT1_ADDITIONAL_CONTROL_HIDE_NON_ZERO_PHYS    (1<<5)
#define MPTSAS_SASIOUNIT1_ADDITIONAL_CONTROL_SATA_ASYNC_NOTIF      (1<<6)
#define MPTSAS_SASIOUNIT1_ADDITIONAL_CONTROL_MULT_PORTS_ILL_SAME_DOMAIN \
 (1<<7)

#define MPTSAS_SASIOUNIT1_MISSING_DEVICE_DELAY_UNITS_16_SEC     (1<<7)
#define MPTSAS_SASIOUNIT1_MISSING_DEVICE_DELAY_SET(x)   ((x) & 0x7F)
#define MPTSAS_SASIOUNIT1_MISSING_DEVICE_DELAY_GET(x)   ((x) & 0x7F)

#define MPTSAS_SASIOUNIT1_PORT_CONFIGURATION_AUTO       (1<<0)
#define MPTSAS_SASIOUNIT1_PORT_CONFIGURATION_IOC1       (1<<2)

#define MPTSAS_SASIOUNIT1_PHY_RX_INVERT                 (1<<0)
#define MPTSAS_SASIOUNIT1_PHY_TX_INVERT                 (1<<1)
#define MPTSAS_SASIOUNIT1_PHY_DISABLE                   (1<<2)

#define MPTSAS_SASIOUNIT1_LINK_RATE_MIN_SET(x)          ((x) & 0xF)
#define MPTSAS_SASIOUNIT1_LINK_RATE_MIN_GET(x)          ((x) & 0xF)
#define MPTSAS_SASIOUNIT1_LINK_RATE_MAX_SET(x)          (((x) & 0xF)<<4)
#define MPTSAS_SASIOUNIT1_LINK_RATE_MAX_GET(x)          ((x >> 4) & 0xF)
#define MPTSAS_SASIOUNIT1_LINK_RATE_15GB                0x8
#define MPTSAS_SASIOUNIT1_LINK_RATE_30GB                0x9

#define MPTSAS_SASIOUNIT1_CTL_PHY_DEVICE_TYPE_SET(x)    ((x) & 0x3)
#define MPTSAS_SASIOUNIT1_CTL_PHY_DEVICE_TYPE_GET(x)    ((x) & 0x3)
#define MPTSAS_SASIOUNIT1_CTL_PHY_DEVICE_TYPE_NO                0x0
#define MPTSAS_SASIOUNIT1_CTL_PHY_DEVICE_TYPE_END               0x1
#define MPTSAS_SASIOUNIT1_CTL_PHY_DEVICE_TYPE_EDGE_EXPANDER     0x2
#define MPTSAS_SASIOUNIT1_CTL_PHY_DEVICE_TYPE_FANOUT_EXPANDER   0x3
#define MPTSAS_SASIOUNIT1_CTL_PHY_DEVICE_SMP_INITIATOR  (1<<4)
#define MPTSAS_SASIOUNIT1_CTL_PHY_DEVICE_STP_INITIATOR  (1<<5)
#define MPTSAS_SASIOUNIT1_CTL_PHY_DEVICE_SSP_INITIATOR  (1<<6)
#define MPTSAS_SASIOUNIT1_CTL_PHY_DEVICE_SMP_TARGET     (1<<8)
#define MPTSAS_SASIOUNIT1_CTL_PHY_DEVICE_STP_TARGET     (1<<9)
#define MPTSAS_SASIOUNIT1_CTL_PHY_DEVICE_SSP_TARGET     (1<<10)
#define MPTSAS_SASIOUNIT1_CTL_PHY_DEVICE_DIRECT_ATTACHED    (1<<11)
#define MPTSAS_SASIOUNIT1_CTL_PHY_DEVICE_LSI            (1<<12)
#define MPTSAS_SASIOUNIT1_CTL_PHY_DEVICE_ATAPI          (1<<13)
#define MPTSAS_SASIOUNIT1_CTL_PHY_DEVICE_SEP            (1<<14)

/**
 * SAS I/O unit page 2 - Read/Write
 */
#pragma pack(1)
typedef struct MptConfigurationPageSASIOUnit2 {
    /** Union. */
    union {
        /** Byte view - variable. */
        uint8_t                                   abPageData[1];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptExtendedConfigurationPageHeader    ExtHeader;
            /** Device numbers per enclosure */
            uint8_t                               NumDevsPerEnclosure;
            /** Boot device wait time */
            uint8_t                               BootDeviceWaitTime;
            /** Reserved */
            uint16_t                              Reserved;
            /** Maximum number of persistent Bus and target ID mappings */
            uint16_t                              MaxPersistentIDs;
            /** Number of persistent IDs used */
            uint16_t                              NumPersistentIDsUsed;
            /** Status */
            uint8_t                               Status;
            /** Flags */
            uint8_t                               Flags;
            /** Maximum number of physical mapped IDs */
            uint16_t                              MaxNumPhysicalMappedIDs;
        } fields;
    } u;
} MptConfigurationPageSASIOUnit2;
#pragma pack()

#define MPTSAS_SASIOUNIT2_STATUS_PERSISTENT_MAP_TBL_FULL       (1<<0)
#define MPTSAS_SASIOUNIT2_STATUS_PERSISTENT_MAP_DISABLED       (1<<1)
#define MPTSAS_SASIOUNIT2_STATUS_PERSISTENT_ENC_DEV_UNMAPPED   (1<<2)
#define MPTSAS_SASIOUNIT2_STATUS_PERSISTENT_DEV_LIMIT_EXCEEDED (1<<3)

#define MPTSAS_SASIOUNIT2_FLAGS_PERSISTENT_MAP_DISABLE          (1<<0)
#define MPTSAS_SASIOUNIT2_FLAGS_PERSISTENT_PHYS_MAP_MODE_SET(x) \
 ((x & 0x7) << 1)
#define MPTSAS_SASIOUNIT2_FLAGS_PERSISTENT_PHYS_MAP_MODE_GET(x) \
 ((x >> 1) & 0x7)
#define MPTSAS_SASIOUNIT2_FLAGS_PERSISTENT_PHYS_MAP_MODE_NO     0x0
#define MPTSAS_SASIOUNIT2_FLAGS_PERSISTENT_PHYS_MAP_MODE_DIRECT_ATTACHED\
 0x1
#define MPTSAS_SASIOUNIT2_FLAGS_PERSISTENT_PHYS_MAP_MODE_ENC    0x2
#define MPTSAS_SASIOUNIT2_FLAGS_PERSISTENT_PHYS_MAP_MODE_HOST   0x7
#define MPTSAS_SASIOUNIT2_FLAGS_RESERVE_TARGET_ID_ZERO          (1<<4)
#define MPTSAS_SASIOUNIT2_FLAGS_START_SLOT_NUMBER_ONE           (1<<5)

/**
 * SAS I/O unit page 3 - Read/Write
 */
#pragma pack(1)
typedef struct MptConfigurationPageSASIOUnit3 {
    /** Union. */
    union {
        /** Byte view - variable. */
        uint8_t                                   abPageData[1];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptExtendedConfigurationPageHeader    ExtHeader;
            /** Reserved */
            uint32_t                          Reserved;
            uint32_t                          MaxInvalidDwordCount;
            uint32_t                          InvalidDwordCountTime;
            uint32_t                          MaxRunningDisparityErrorCount;
            uint32_t                          RunningDisparityErrorTime;
            uint32_t                          MaxLossDwordSynchCount;
            uint32_t                          LossDwordSynchCountTime;
            uint32_t                          MaxPhysResetProblemCount;
            uint32_t                          PhyResetProblemTime;
        } fields;
    } u;
} MptConfigurationPageSASIOUnit3;
#pragma pack()

/**
 * SAS PHY page 0 - Readonly
 */
#pragma pack(1)
typedef struct MptConfigurationPageSASPHY0 {
    /** Union. */
    union {
        /** Byte view - variable. */
        uint8_t                                   abPageData[1];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptExtendedConfigurationPageHeader    ExtHeader;
            /** Owner dev handle. */
            uint16_t                              OwnerDevHandle;
            /** Reserved */
            uint16_t                              Reserved0;
            /** SAS address */
            uint64_t                              SASAddress;
            /** Attached device handle */
            uint16_t                              AttachedDevHandle;
            /** Attached phy identifier */
            uint8_t                               AttachedPhyIdentifier;
            /** Reserved */
            uint8_t                               Reserved1;
            /** Attached device information */
            uint32_t                              AttachedDeviceInfo;
            /** Programmed link rate */
            uint8_t                               ProgrammedLinkRate;
            /** Hardware link rate */
            uint8_t                               HwLinkRate;
            /** Change count */
            uint8_t                               ChangeCount;
            /** Flags */
            uint8_t                               Flags;
            /** Phy information */
            uint32_t                              PhyInfo;
        } fields;
    } u;
} MptConfigurationPageSASPHY0;
#pragma pack()

#define MPTSAS_SASPHY0_DEV_INFO_DEVICE_TYPE_SET(x)          ((x) & 0x3)
#define MPTSAS_SASPHY0_DEV_INFO_DEVICE_TYPE_GET(x)          ((x) & 0x3)
#define MPTSAS_SASPHY0_DEV_INFO_DEVICE_TYPE_NO              0x0
#define MPTSAS_SASPHY0_DEV_INFO_DEVICE_TYPE_END             0x1
#define MPTSAS_SASPHY0_DEV_INFO_DEVICE_TYPE_EDGE_EXPANDER   0x2
#define MPTSAS_SASPHY0_DEV_INFO_DEVICE_TYPE_FANOUT_EXPANDER 0x3
#define MPTSAS_SASPHY0_DEV_INFO_DEVICE_SMP_INITIATOR        (1<<4)
#define MPTSAS_SASPHY0_DEV_INFO_DEVICE_STP_INITIATOR        (1<<5)
#define MPTSAS_SASPHY0_DEV_INFO_DEVICE_SSP_INITIATOR        (1<<6)
#define MPTSAS_SASPHY0_DEV_INFO_DEVICE_SMP_TARGET           (1<<8)
#define MPTSAS_SASPHY0_DEV_INFO_DEVICE_STP_TARGET           (1<<9)
#define MPTSAS_SASPHY0_DEV_INFO_DEVICE_SSP_TARGET           (1<<10)
#define MPTSAS_SASPHY0_DEV_INFO_DEVICE_DIRECT_ATTACHED      (1<<11)
#define MPTSAS_SASPHY0_DEV_INFO_DEVICE_LSI                  (1<<12)
#define MPTSAS_SASPHY0_DEV_INFO_DEVICE_ATAPI                (1<<13)
#define MPTSAS_SASPHY0_DEV_INFO_DEVICE_SEP                  (1<<14)

/**
 * SAS PHY page 1 - Readonly
 */
#pragma pack(1)
typedef struct MptConfigurationPageSASPHY1 {
    /** Union. */
    union {
        /** Byte view - variable. */
        uint8_t                                   abPageData[1];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptExtendedConfigurationPageHeader    ExtHeader;
            /** Reserved */
            uint32_t                              Reserved0;
            uint32_t                              InvalidDwordCound;
            uint32_t                              RunningDisparityErrorCount;
            uint32_t                              LossDwordSynchCount;
            uint32_t                              PhyResetProblemCount;
        } fields;
    } u;
} MptConfigurationPageSASPHY1;
#pragma pack()

/**
 * SAS Device page 0 - Readonly
 */
#pragma pack(1)
typedef struct MptConfigurationPageSASDevice0 {
    /** Union. */
    union {
        /** Byte view - variable. */
        uint8_t                                   abPageData[1];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptExtendedConfigurationPageHeader    ExtHeader;
            /** Slot number */
            uint16_t                              Slot;
            /** Enclosure handle. */
            uint16_t                              EnclosureHandle;
            /** SAS address */
            uint64_t                              SASAddress;
            /** Parent device handle */
            uint16_t                              ParentDevHandle;
            /** Phy number */
            uint8_t                               PhyNum;
            /** Access status */
            uint8_t                               AccessStatus;
            /** Device handle */
            uint16_t                              DevHandle;
            /** Target ID */
            uint8_t                               TargetID;
            /** Bus */
            uint8_t                               Bus;
            /** Device info */
            uint32_t                              DeviceInfo;
            /** Flags */
            uint16_t                              Flags;
            /** Physical port */
            uint8_t                               PhysicalPort;
            /** Reserved */
            uint8_t                               Reserved0;
        } fields;
    } u;
} MptConfigurationPageSASDevice0;
#pragma pack()

#define MPTSAS_SASDEVICE0_STATUS_NO_ERRORS                 (0x00)

#define MPTSAS_SASDEVICE0_DEV_INFO_DEVICE_TYPE_SET(x)      ((x) & 0x3)
#define MPTSAS_SASDEVICE0_DEV_INFO_DEVICE_TYPE_GET(x)      ((x) & 0x3)
#define MPTSAS_SASDEVICE0_DEV_INFO_DEVICE_TYPE_NO              0x0
#define MPTSAS_SASDEVICE0_DEV_INFO_DEVICE_TYPE_END             0x1
#define MPTSAS_SASDEVICE0_DEV_INFO_DEVICE_TYPE_EDGE_EXPANDER   0x2
#define MPTSAS_SASDEVICE0_DEV_INFO_DEVICE_TYPE_FANOUT_EXPANDER 0x3
#define MPTSAS_SASDEVICE0_DEV_INFO_DEVICE_SMP_INITIATOR        (1<<4)
#define MPTSAS_SASDEVICE0_DEV_INFO_DEVICE_STP_INITIATOR        (1<<5)
#define MPTSAS_SASDEVICE0_DEV_INFO_DEVICE_SSP_INITIATOR        (1<<6)
#define MPTSAS_SASDEVICE0_DEV_INFO_DEVICE_SMP_TARGET           (1<<8)
#define MPTSAS_SASDEVICE0_DEV_INFO_DEVICE_STP_TARGET           (1<<9)
#define MPTSAS_SASDEVICE0_DEV_INFO_DEVICE_SSP_TARGET           (1<<10)
#define MPTSAS_SASDEVICE0_DEV_INFO_DEVICE_DIRECT_ATTACHED      (1<<11)
#define MPTSAS_SASDEVICE0_DEV_INFO_DEVICE_LSI                  (1<<12)
#define MPTSAS_SASDEVICE0_DEV_INFO_DEVICE_ATAPI                (1<<13)
#define MPTSAS_SASDEVICE0_DEV_INFO_DEVICE_SEP                  (1<<14)

#define MPTSAS_SASDEVICE0_FLAGS_DEVICE_PRESENT                 (1<<0)
#define MPTSAS_SASDEVICE0_FLAGS_DEVICE_MAPPED_TO_BUS_AND_TARGET_ID \
 (1<<(1))
#define MPTSAS_SASDEVICE0_FLAGS_DEVICE_MAPPING_PERSISTENT \
 (1<<(2))

/**
 * SAS Device page 1 - Readonly
 */
#pragma pack(1)
typedef struct MptConfigurationPageSASDevice1 {
    /** Union. */
    union {
        /** Byte view - variable. */
        uint8_t                                   abPageData[1];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptExtendedConfigurationPageHeader    ExtHeader;
            /** Reserved */
            uint32_t                              Reserved0;
            /** SAS address */
            uint64_t                              SASAddress;
            /** Reserved */
            uint32_t                              Reserved;
            /** Device handle */
            uint16_t                              DevHandle;
            /** Target ID */
            uint8_t                               TargetID;
            /** Bus */
            uint8_t                               Bus;
            /** Initial REgister device FIS */
            uint32_t                              au32InitialRegDeviceFIS[5];
        } fields;
    } u;
} MptConfigurationPageSASDevice1;
#pragma pack()

/**
 * SAS Device page 2 - Read/Write persistent
 */
#pragma pack(1)
typedef struct MptConfigurationPageSASDevice2 {
    /** Union. */
    union {
        /** Byte view - variable. */
        uint8_t                                   abPageData[1];
        /** Field view. */
        struct {
            /** The omnipresent header. */
            MptExtendedConfigurationPageHeader    ExtHeader;
            /** Physical identifier */
            uint64_t                              SASAddress;
            /** Enclosure mapping */
            uint32_t                              EnclosureMapping;
        } fields;
    } u;
} MptConfigurationPageSASDevice2;
#pragma pack()

/**
 * A device entitiy containing all pages.
 */
typedef struct MptSASDevice {
    /** Pointer to the next device if any. */
    struct MptSASDevice            *pNext;
    /** Pointer to the previous device if any. */
    struct MptSASDevice            *pPrev;

    MptConfigurationPageSASDevice0  SASDevicePage0;
    MptConfigurationPageSASDevice1  SASDevicePage1;
    MptConfigurationPageSASDevice2  SASDevicePage2;
} MptSASDevice;

typedef struct MptPHY {
    MptConfigurationPageSASPHY0     SASPHYPage0;
    MptConfigurationPageSASPHY1     SASPHYPage1;
} MptPHY;

#pragma pack(1)
typedef struct MptConfigurationPagesSas {
} MptConfigurationPagesSas;
#pragma pack()

/**
 * Structure of all supported pages for both controllers.
 */
struct MptConfigurationPagesSupported {
    MptConfigurationPageManufacturing0  ManufacturingPage0;
    MptConfigurationPageManufacturing1  ManufacturingPage1;
    MptConfigurationPageManufacturing2  ManufacturingPage2;
    MptConfigurationPageManufacturing3  ManufacturingPage3;
    MptConfigurationPageManufacturing4  ManufacturingPage4;
    MptConfigurationPageManufacturing5  ManufacturingPage5;
    MptConfigurationPageManufacturing6  ManufacturingPage6;
    MptConfigurationPageManufacturing8  ManufacturingPage8;
    MptConfigurationPageManufacturing9  ManufacturingPage9;
    MptConfigurationPageManufacturing10 ManufacturingPage10;
    MptConfigurationPageIOUnit0         IOUnitPage0;
    MptConfigurationPageIOUnit1         IOUnitPage1;
    MptConfigurationPageIOUnit2         IOUnitPage2;
    MptConfigurationPageIOUnit3         IOUnitPage3;
    MptConfigurationPageIOUnit4         IOUnitPage4;
    MptConfigurationPageIOC0            IOCPage0;
    MptConfigurationPageIOC1            IOCPage1;
    MptConfigurationPageIOC2            IOCPage2;
    MptConfigurationPageIOC3            IOCPage3;
    MptConfigurationPageIOC4            IOCPage4;
    MptConfigurationPageIOC6            IOCPage6;
    /* BIOS page 0 is not described */
    MptConfigurationPageBIOS1           BIOSPage1;
    MptConfigurationPageBIOS2           BIOSPage2;
    /* BIOS page 3 is not described */
    MptConfigurationPageBIOS4           BIOSPage4;

    /** Controller dependent data. */

    /** Pointer to the manufacturing page 7 */
    MptConfigurationPageManufacturing7  ManufacturingPage7;
    /** Pointer to the I/O unit page 0 */
    MptConfigurationPageSASIOUnit0      SASIOUnitPage0;
    /** Pointer to the I/O unit page 1 */
    MptConfigurationPageSASIOUnit1      SASIOUnitPage1;
    /** I/O unit page 2 */
    MptConfigurationPageSASIOUnit2      SASIOUnitPage2;
    /** I/O unit page 3 */
    MptConfigurationPageSASIOUnit3      SASIOUnitPage3;

    /** Pointer to an array of per PHYS pages. */
    MptPHY                              aPHYs[MPTSAS_NUM_PORTS];

    /** Pointer to the first SAS device. */
    MptSASDevice                       *pSASDeviceHead;
    /** Pointer to the last SAS device. */
    MptSASDevice                       *pSASDeviceTail;
};

/**
 * Initializes a page header.
 */
#define MPT_CONFIG_PAGE_HEADER_INIT(pg, type, nr, flags) \
    (pg)->u.fields.Header.PageType   = flags; \
    (pg)->u.fields.Header.PageNumber = nr; \
    (pg)->u.fields.Header.PageLength = sizeof(type) / 4

#define MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(pg, type, nr, flags) \
    MPT_CONFIG_PAGE_HEADER_INIT(pg, type, nr, flags | \
        MPT_CONFIGURATION_PAGE_TYPE_MANUFACTURING)

#define MPT_CONFIG_PAGE_HEADER_INIT_IO_UNIT(pg, type, nr, flags) \
    MPT_CONFIG_PAGE_HEADER_INIT(pg, type, nr, flags | \
        MPT_CONFIGURATION_PAGE_TYPE_IO_UNIT)

#define MPT_CONFIG_PAGE_HEADER_INIT_IOC(pg, type, nr, flags) \
    MPT_CONFIG_PAGE_HEADER_INIT(pg, type, nr, flags | \
        MPT_CONFIGURATION_PAGE_TYPE_IOC)

#define MPT_CONFIG_PAGE_HEADER_INIT_BIOS(pg, type, nr, flags) \
    MPT_CONFIG_PAGE_HEADER_INIT(pg, type, nr, flags | \
        MPT_CONFIGURATION_PAGE_TYPE_BIOS)

/**
 * Initializes a extended page header.
 */
#define MPT_CONFIG_EXTENDED_PAGE_HEADER_INIT(pg, cb, nr, flags, exttype) \
    (pg)->u.fields.ExtHeader.PageType   = flags | \
        MPT_CONFIGURATION_PAGE_TYPE_EXTENDED; \
    (pg)->u.fields.ExtHeader.PageNumber = nr; \
    (pg)->u.fields.ExtHeader.ExtPageType = exttype; \
    (pg)->u.fields.ExtHeader.ExtPageLength = cb / 4

void mptsas_config_pages_free(MPTSASState *s)
{

    if (s->config_pages) {
        /* Destroy device list if we emulate a SAS controller. */
        MptSASDevice *pSASDeviceCurr = s->config_pages->pSASDeviceHead;

        while (pSASDeviceCurr) {
            MptSASDevice *pFree = pSASDeviceCurr;

            pSASDeviceCurr = pSASDeviceCurr->pNext;
            g_free(pFree);
        }

        g_free(s->config_pages);
    }
}

static void mptsas_init_config_pages_sas(MPTSASState *s)
{
    /* Manufacturing Page 7 - Connector settings. */
    MptConfigurationPageManufacturing7 *pManufacturingPage7 =
        &s->config_pages->ManufacturingPage7;
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(pManufacturingPage7, 0, 7,
                      MPT_CONFIGURATION_PAGE_ATTRIBUTE_PERSISTENT_READONLY);
    /* Set size manually. */
    pManufacturingPage7->u.fields.Header.PageLength =
        sizeof(MptConfigurationPageManufacturing7) / 4;
    pManufacturingPage7->u.fields.NumPhys = MPTSAS_NUM_PORTS;

    /* SAS I/O unit page 0 - Port specific information. */
    MptConfigurationPageSASIOUnit0 *pSASPage0 =
        &s->config_pages->SASIOUnitPage0;

    MPT_CONFIG_EXTENDED_PAGE_HEADER_INIT(pSASPage0, sizeof(MptConfigurationPageSASIOUnit0),
                             0, MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY,
                             MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASIOUNIT);
    pSASPage0->u.fields.NumPhys = MPTSAS_NUM_PORTS;

    /* SAS I/O unit page 1 - Port specific settings. */
    MptConfigurationPageSASIOUnit1 *pSASPage1 =
        &s->config_pages->SASIOUnitPage1;

    MPT_CONFIG_EXTENDED_PAGE_HEADER_INIT(pSASPage1, sizeof(MptConfigurationPageSASIOUnit1),
                             1, MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE,
                             MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASIOUNIT);
    pSASPage1->u.fields.NumPhys = pSASPage0->u.fields.NumPhys;
    pSASPage1->u.fields.ControlFlags = 0;
    pSASPage1->u.fields.AdditionalControlFlags = 0;

    /* SAS I/O unit page 2 - Port specific information. */
    s->config_pages->SASIOUnitPage2.u.fields.ExtHeader.PageType       =
        MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY
         | MPT_CONFIGURATION_PAGE_TYPE_EXTENDED;
    s->config_pages->SASIOUnitPage2.u.fields.ExtHeader.PageNumber     = 2;
    s->config_pages->SASIOUnitPage2.u.fields.ExtHeader.ExtPageType    =
        MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASIOUNIT;
    s->config_pages->SASIOUnitPage2.u.fields.ExtHeader.ExtPageLength =
        sizeof(MptConfigurationPageSASIOUnit2) / 4;

    /* SAS I/O unit page 3 - Port specific information. */
    s->config_pages->SASIOUnitPage3.u.fields.ExtHeader.PageType       =
        MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY
         | MPT_CONFIGURATION_PAGE_TYPE_EXTENDED;
    s->config_pages->SASIOUnitPage3.u.fields.ExtHeader.PageNumber     = 3;
    s->config_pages->SASIOUnitPage3.u.fields.ExtHeader.ExtPageType    =
        MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASIOUNIT;
    s->config_pages->SASIOUnitPage3.u.fields.ExtHeader.ExtPageLength =
        sizeof(MptConfigurationPageSASIOUnit3) / 4;

    /* Initialize the PHY configuration */
    unsigned i;
    unsigned handle = 0;

    for (i = 0; i < MPTSAS_NUM_PORTS; i++) {
        MptPHY *pPHYPages = &s->config_pages->aPHYs[i];
        uint16_t ControllerHandle = ++handle;

        pManufacturingPage7->u.fields.aPHY[i].Location =
                MPTSAS_MANUFACTURING7_LOCATION_AUTO;

        pSASPage0->u.fields.aPHY[i].Port      = i;
        pSASPage0->u.fields.aPHY[i].PortFlags = 0;
        pSASPage0->u.fields.aPHY[i].PhyFlags  = 0;
        pSASPage0->u.fields.aPHY[i].NegotiatedLinkRate =
                MPTSAS_SASIOUNIT0_NEGOTIATED_RATE_FAILED;
        pSASPage0->u.fields.aPHY[i].ControllerPhyDeviceInfo =
                MPTSAS_SASIOUNIT0_DEVICE_TYPE_SET(
                    MPTSAS_SASIOUNIT0_DEVICE_TYPE_NO);
        pSASPage0->u.fields.aPHY[i].ControllerDevHandle =
                ControllerHandle;
        pSASPage0->u.fields.aPHY[i].AttachedDevHandle = 0;
        pSASPage0->u.fields.aPHY[i].DiscoveryStatus = 0;

        pSASPage1->u.fields.aPHY[i].Port           = i;
        pSASPage1->u.fields.aPHY[i].PortFlags      = 0;
        pSASPage1->u.fields.aPHY[i].PhyFlags       = 0;
        pSASPage1->u.fields.aPHY[i].MaxMinLinkRate =
                MPTSAS_SASIOUNIT1_LINK_RATE_MIN_SET(
                    MPTSAS_SASIOUNIT1_LINK_RATE_15GB)
                   | MPTSAS_SASIOUNIT1_LINK_RATE_MAX_SET(
                    MPTSAS_SASIOUNIT1_LINK_RATE_30GB);
        pSASPage1->u.fields.aPHY[i].ControllerPhyDeviceInfo =
                MPTSAS_SASIOUNIT0_DEVICE_TYPE_SET(
                    MPTSAS_SASIOUNIT0_DEVICE_TYPE_NO);

        /* SAS PHY page 0. */
        pPHYPages->SASPHYPage0.u.fields.ExtHeader.PageType       =
                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY
                  | MPT_CONFIGURATION_PAGE_TYPE_EXTENDED;
        pPHYPages->SASPHYPage0.u.fields.ExtHeader.PageNumber     = 0;
        pPHYPages->SASPHYPage0.u.fields.ExtHeader.ExtPageType    =
                MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASPHYS;
        pPHYPages->SASPHYPage0.u.fields.ExtHeader.ExtPageLength =
                sizeof(MptConfigurationPageSASPHY0) / 4;
        pPHYPages->SASPHYPage0.u.fields.AttachedPhyIdentifier    = i;
        pPHYPages->SASPHYPage0.u.fields.AttachedDeviceInfo      =
                MPTSAS_SASPHY0_DEV_INFO_DEVICE_TYPE_SET(
                    MPTSAS_SASPHY0_DEV_INFO_DEVICE_TYPE_NO);
        pPHYPages->SASPHYPage0.u.fields.ProgrammedLinkRate       =
                MPTSAS_SASIOUNIT1_LINK_RATE_MIN_SET(
                    MPTSAS_SASIOUNIT1_LINK_RATE_15GB)
                 | MPTSAS_SASIOUNIT1_LINK_RATE_MAX_SET(
                    MPTSAS_SASIOUNIT1_LINK_RATE_30GB);
        pPHYPages->SASPHYPage0.u.fields.HwLinkRate               =
                MPTSAS_SASIOUNIT1_LINK_RATE_MIN_SET(
                    MPTSAS_SASIOUNIT1_LINK_RATE_15GB)
                 | MPTSAS_SASIOUNIT1_LINK_RATE_MAX_SET(
                    MPTSAS_SASIOUNIT1_LINK_RATE_30GB);

        /* SAS PHY page 1. */
        pPHYPages->SASPHYPage1.u.fields.ExtHeader.PageType       =
                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY
                 | MPT_CONFIGURATION_PAGE_TYPE_EXTENDED;
        pPHYPages->SASPHYPage1.u.fields.ExtHeader.PageNumber     = 1;
        pPHYPages->SASPHYPage1.u.fields.ExtHeader.ExtPageType    =
                MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASPHYS;
        pPHYPages->SASPHYPage1.u.fields.ExtHeader.ExtPageLength =
                sizeof(MptConfigurationPageSASPHY1) / 4;

        /* Settings for present devices. */
        if (scsi_device_find(&s->bus, 0, i, 0)) {
            uint16_t DeviceHandle = ++handle;
            MptSASDevice *pSASDevice =
                (MptSASDevice*)g_malloc0(sizeof(MptSASDevice));

            pSASPage0->u.fields.aPHY[i].NegotiatedLinkRate       =
                MPTSAS_SASIOUNIT0_NEGOTIATED_RATE_SET(
                    MPTSAS_SASIOUNIT0_NEGOTIATED_RATE_30GB);
            pSASPage0->u.fields.aPHY[i].ControllerPhyDeviceInfo =
                MPTSAS_SASIOUNIT0_DEVICE_TYPE_SET(
                    MPTSAS_SASIOUNIT0_DEVICE_TYPE_END)
                 | MPTSAS_SASIOUNIT0_DEVICE_SSP_TARGET;
            pSASPage0->u.fields.aPHY[i].AttachedDevHandle       =
                DeviceHandle;
            pSASPage1->u.fields.aPHY[i].ControllerPhyDeviceInfo =
                MPTSAS_SASIOUNIT0_DEVICE_TYPE_SET(
                    MPTSAS_SASIOUNIT0_DEVICE_TYPE_END)
                 | MPTSAS_SASIOUNIT0_DEVICE_SSP_TARGET;
            pSASPage0->u.fields.aPHY[i].ControllerDevHandle     =
                DeviceHandle;

            pPHYPages->SASPHYPage0.u.fields.AttachedDeviceInfo  =
                MPTSAS_SASPHY0_DEV_INFO_DEVICE_TYPE_SET(
                    MPTSAS_SASPHY0_DEV_INFO_DEVICE_TYPE_END);
            pPHYPages->SASPHYPage0.u.fields.SASAddress             =
                s->sas_addr;
            pPHYPages->SASPHYPage0.u.fields.OwnerDevHandle      =
                DeviceHandle;
            pPHYPages->SASPHYPage0.u.fields.AttachedDevHandle   =
                DeviceHandle;

            /* SAS device page 0. */
            pSASDevice->SASDevicePage0.u.fields.ExtHeader.PageType       =
                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY
                 | MPT_CONFIGURATION_PAGE_TYPE_EXTENDED;
            pSASDevice->SASDevicePage0.u.fields.ExtHeader.PageNumber     = 0;
            pSASDevice->SASDevicePage0.u.fields.ExtHeader.ExtPageType    =
                MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASDEVICE;
            pSASDevice->SASDevicePage0.u.fields.ExtHeader.ExtPageLength =
                sizeof(MptConfigurationPageSASDevice0) / 4;
            pSASDevice->SASDevicePage0.u.fields.SASAddress                 =
                s->sas_addr;
            pSASDevice->SASDevicePage0.u.fields.ParentDevHandle         =
                ControllerHandle;
            pSASDevice->SASDevicePage0.u.fields.PhyNum                   = i;
            pSASDevice->SASDevicePage0.u.fields.AccessStatus =
                MPTSAS_SASDEVICE0_STATUS_NO_ERRORS;
            pSASDevice->SASDevicePage0.u.fields.DevHandle = DeviceHandle;
            pSASDevice->SASDevicePage0.u.fields.TargetID                 = i;
            pSASDevice->SASDevicePage0.u.fields.Bus                      = 0;
            pSASDevice->SASDevicePage0.u.fields.DeviceInfo              =
                MPTSAS_SASPHY0_DEV_INFO_DEVICE_TYPE_SET(
                    MPTSAS_SASPHY0_DEV_INFO_DEVICE_TYPE_END)
                     | MPTSAS_SASIOUNIT0_DEVICE_SSP_TARGET;
            pSASDevice->SASDevicePage0.u.fields.Flags                   =
             MPTSAS_SASDEVICE0_FLAGS_DEVICE_PRESENT
             | MPTSAS_SASDEVICE0_FLAGS_DEVICE_MAPPED_TO_BUS_AND_TARGET_ID
             | MPTSAS_SASDEVICE0_FLAGS_DEVICE_MAPPING_PERSISTENT;
            pSASDevice->SASDevicePage0.u.fields.PhysicalPort             = i;

            /* SAS device page 1. */
            pSASDevice->SASDevicePage1.u.fields.ExtHeader.PageType       =
                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY
                     | MPT_CONFIGURATION_PAGE_TYPE_EXTENDED;
            pSASDevice->SASDevicePage1.u.fields.ExtHeader.PageNumber     = 1;
            pSASDevice->SASDevicePage1.u.fields.ExtHeader.ExtPageType    =
                MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASDEVICE;
            pSASDevice->SASDevicePage1.u.fields.ExtHeader.ExtPageLength =
                sizeof(MptConfigurationPageSASDevice1) / 4;
            pSASDevice->SASDevicePage1.u.fields.SASAddress = s->sas_addr;
            pSASDevice->SASDevicePage1.u.fields.DevHandle = DeviceHandle;
            pSASDevice->SASDevicePage1.u.fields.TargetID                 = i;
            pSASDevice->SASDevicePage1.u.fields.Bus                      = 0;

            /* SAS device page 2. */
            pSASDevice->SASDevicePage2.u.fields.ExtHeader.PageType       =
                        MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY
                          | MPT_CONFIGURATION_PAGE_TYPE_EXTENDED;
            pSASDevice->SASDevicePage2.u.fields.ExtHeader.PageNumber     = 2;
            pSASDevice->SASDevicePage2.u.fields.ExtHeader.ExtPageType    =
                        MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASDEVICE;
            pSASDevice->SASDevicePage2.u.fields.ExtHeader.ExtPageLength =
                        sizeof(MptConfigurationPageSASDevice2) / 4;
            pSASDevice->SASDevicePage2.u.fields.SASAddress                 =
                        s->sas_addr;

            /* Link into device list. */
            if (!s->config_pages->pSASDeviceHead) {
                s->config_pages->pSASDeviceHead = pSASDevice;
                s->config_pages->pSASDeviceTail = pSASDevice;
            } else {
                pSASDevice->pPrev = s->config_pages->pSASDeviceTail;
                s->config_pages->pSASDeviceTail->pNext = pSASDevice;
                s->config_pages->pSASDeviceTail = pSASDevice;
            }
        }
    }
}

void mptsas_init_config_pages(MPTSASState *s)
{
    PCIDeviceClass *pci_class = PCI_DEVICE_GET_CLASS(s);

    /* Initialize the common pages. */
    MptConfigurationPagesSupported *pPages =
        (MptConfigurationPagesSupported*)g_malloc0(
                sizeof(MptConfigurationPagesSupported));

    s->config_pages = pPages;

    /* Clear everything first. */
    memset(pPages, 0, sizeof(MptConfigurationPagesSupported));

    /* Manufacturing Page 0. */
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(&pPages->ManufacturingPage0,
                      MptConfigurationPageManufacturing0, 0,
                      MPT_CONFIGURATION_PAGE_ATTRIBUTE_PERSISTENT_READONLY);
    strncpy((char *)pPages->ManufacturingPage0.u.fields.abChipName,
                                                    "QEMU MPT Fusion", 16);
    strncpy((char *)pPages->ManufacturingPage0.u.fields.abChipRevision,
                                                    "1.0", 8);
    strncpy((char *)pPages->ManufacturingPage0.u.fields.abBoardName,
                                                    "QEMU MPT Fusion", 16);
    strncpy((char *)pPages->ManufacturingPage0.u.fields.abBoardAssembly,
                                                    "QEMU", 8);
    strncpy((char *)pPages->ManufacturingPage0.u.fields.abBoardTracerNumber,
                                                    "DEADBEEFDEADBEEF", 16);

    /* Manufacturing Page 1 - Leave it 0 for now. */
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(&pPages->ManufacturingPage1,
                      MptConfigurationPageManufacturing1, 1,
                      MPT_CONFIGURATION_PAGE_ATTRIBUTE_PERSISTENT_READONLY);

    /* Manufacturing Page 2. */
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(&pPages->ManufacturingPage2,
                      MptConfigurationPageManufacturing2, 2,
                      MPT_CONFIGURATION_PAGE_ATTRIBUTE_PERSISTENT_READONLY);

    pPages->ManufacturingPage2.u.fields.PCIDeviceID =
        pci_class->device_id;
    pPages->ManufacturingPage2.u.fields.PCIRevisionID =
        pci_class->revision;

    /* Manufacturing Page 3. */
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(&pPages->ManufacturingPage3,
                      MptConfigurationPageManufacturing3, 3,
                      MPT_CONFIGURATION_PAGE_ATTRIBUTE_PERSISTENT_READONLY);

    pPages->ManufacturingPage3.u.fields.PCIDeviceID =
        pci_class->device_id;
    pPages->ManufacturingPage3.u.fields.PCIRevisionID =
        pci_class->revision;

    /* Manufacturing Page 4 - Leave it 0 for now. */
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(&pPages->ManufacturingPage4,
                      MptConfigurationPageManufacturing4, 4,
                      MPT_CONFIGURATION_PAGE_ATTRIBUTE_PERSISTENT_READONLY);

    /* Manufacturing Page 5 - WWID settings. */
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(&pPages->ManufacturingPage5,
                      MptConfigurationPageManufacturing5, 5,
                      MPT_CONFIGURATION_PAGE_ATTRIBUTE_PERSISTENT_READONLY);

    /* Manufacturing Page 6 - Product specific settings. */
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(&pPages->ManufacturingPage6,
                                  MptConfigurationPageManufacturing6, 6,
                                  MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE);

    /* Manufacturing Page 8 -  Product specific settings. */
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(&pPages->ManufacturingPage8,
                                  MptConfigurationPageManufacturing8, 8,
                                  MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE);

    /* Manufacturing Page 9 -  Product specific settings. */
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(&pPages->ManufacturingPage9,
                                  MptConfigurationPageManufacturing9, 9,
                                  MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE);

    /* Manufacturing Page 10 -  Product specific settings. */
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(&pPages->ManufacturingPage10,
                                  MptConfigurationPageManufacturing10, 10,
                                  MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE);

    /* I/O Unit page 0. */
    MPT_CONFIG_PAGE_HEADER_INIT_IO_UNIT(&pPages->IOUnitPage0,
                                MptConfigurationPageIOUnit0, 0,
                                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY);
    pPages->IOUnitPage0.u.fields.UniqueIdentifier = 0xcafe;

    /* I/O Unit page 1. */
    MPT_CONFIG_PAGE_HEADER_INIT_IO_UNIT(&pPages->IOUnitPage1,
                                MptConfigurationPageIOUnit1, 1,
                                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY);
    pPages->IOUnitPage1.u.fields.SingleFunction         = true;
    pPages->IOUnitPage1.u.fields.AllPathsMapped         = false;
    pPages->IOUnitPage1.u.fields.IntegratedRAIDDisabled = true;
    pPages->IOUnitPage1.u.fields.BitAccessForced      = false;

    /* I/O Unit page 2. */
    MPT_CONFIG_PAGE_HEADER_INIT_IO_UNIT(&pPages->IOUnitPage2,
                                MptConfigurationPageIOUnit2, 2,
                                MPT_CONFIGURATION_PAGE_ATTRIBUTE_PERSISTENT);
    pPages->IOUnitPage2.u.fields.PauseOnError       = false;
    pPages->IOUnitPage2.u.fields.VerboseModeEnabled = false;
    pPages->IOUnitPage2.u.fields.DisableColorVideo  = false;
    pPages->IOUnitPage2.u.fields.NotHookInt40h      = false;
    pPages->IOUnitPage2.u.fields.BIOSVersion      = 0xdeadbeef;
    pPages->IOUnitPage2.u.fields.aAdapterOrder[0].AdapterEnabled = true;
    pPages->IOUnitPage2.u.fields.aAdapterOrder[0].AdapterEmbedded = true;
    pPages->IOUnitPage2.u.fields.aAdapterOrder[0].PCIBusNumber = 0;
    pPages->IOUnitPage2.u.fields.aAdapterOrder[0].PCIDevFn = s->dev.devfn;

    /* I/O Unit page 3. */
    MPT_CONFIG_PAGE_HEADER_INIT_IO_UNIT(&pPages->IOUnitPage3,
                                MptConfigurationPageIOUnit3, 3,
                                MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE);
    pPages->IOUnitPage3.u.fields.GPIOCount = 0;

    /* I/O Unit page 4. */
    MPT_CONFIG_PAGE_HEADER_INIT_IO_UNIT(&pPages->IOUnitPage4,
                                MptConfigurationPageIOUnit4, 4,
                                MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE);

    /* IOC page 0. */
    MPT_CONFIG_PAGE_HEADER_INIT_IOC(&pPages->IOCPage0,
                                MptConfigurationPageIOC0, 0,
                                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY);
    pPages->IOCPage0.u.fields.TotalNVStore      = 0;
    pPages->IOCPage0.u.fields.FreeNVStore       = 0;

    pPages->IOCPage0.u.fields.VendorId          =
        pci_class->vendor_id;
    pPages->IOCPage0.u.fields.DeviceId          =
        pci_class->device_id;
    pPages->IOCPage0.u.fields.RevisionId         =
        pci_class->revision;
    pPages->IOCPage0.u.fields.ClassCode         =
        pci_class->class_id;
    pPages->IOCPage0.u.fields.SubsystemVendorId =
        pci_class->subsystem_vendor_id;
    pPages->IOCPage0.u.fields.SubsystemId       =
        pci_class->subsystem_id;

    /* IOC page 1. */
    MPT_CONFIG_PAGE_HEADER_INIT_IOC(&pPages->IOCPage1,
                            MptConfigurationPageIOC1, 1,
                            MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE);
    pPages->IOCPage1.u.fields.ReplyCoalescingEnabled = false;
    pPages->IOCPage1.u.fields.CoalescingTimeout    = 0;
    pPages->IOCPage1.u.fields.CoalescingDepth       = 0;

    /* IOC page 2. */
    MPT_CONFIG_PAGE_HEADER_INIT_IOC(&pPages->IOCPage2,
                                MptConfigurationPageIOC2, 2,
                                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY);
    /* Everything else here is 0. */

    /* IOC page 3. */
    MPT_CONFIG_PAGE_HEADER_INIT_IOC(&pPages->IOCPage3,
                                MptConfigurationPageIOC3, 3,
                                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY);
    /* Everything else here is 0. */

    /* IOC page 4. */
    MPT_CONFIG_PAGE_HEADER_INIT_IOC(&pPages->IOCPage4,
                                MptConfigurationPageIOC4, 4,
                                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY);
    /* Everything else here is 0. */

    /* IOC page 6. */
    MPT_CONFIG_PAGE_HEADER_INIT_IOC(&pPages->IOCPage6,
                                MptConfigurationPageIOC6, 6,
                                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY);
    /* Everything else here is 0. */

    /* BIOS page 1. */
    MPT_CONFIG_PAGE_HEADER_INIT_BIOS(&pPages->BIOSPage1,
                                 MptConfigurationPageBIOS1, 1,
                                 MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE);

    /* BIOS page 2. */
    MPT_CONFIG_PAGE_HEADER_INIT_BIOS(&pPages->BIOSPage2,
                                 MptConfigurationPageBIOS2, 2,
                                 MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE);

    /* BIOS page 4. */
    MPT_CONFIG_PAGE_HEADER_INIT_BIOS(&pPages->BIOSPage4,
                                 MptConfigurationPageBIOS4, 4,
                                 MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE);

    mptsas_init_config_pages_sas(s);
}

static int mptsas_config_unit_page(MPTSASState *pLsiLogic,
                            MptConfigurationPagesSupported *pPages,
                            uint8_t PageNumber,
                            MptConfigurationPageHeader **ppPageHeader,
                            uint8_t **ppbPageData, size_t *pcbPage)
{
    int rc = 0;

    switch (PageNumber) {
    case 0:
        *ppPageHeader = &pPages->IOUnitPage0.u.fields.Header;
        *ppbPageData  =  pPages->IOUnitPage0.u.abPageData;
        *pcbPage      = sizeof(pPages->IOUnitPage0);
        break;
    case 1:
        *ppPageHeader = &pPages->IOUnitPage1.u.fields.Header;
        *ppbPageData  =  pPages->IOUnitPage1.u.abPageData;
        *pcbPage      = sizeof(pPages->IOUnitPage1);
        break;
    case 2:
        *ppPageHeader = &pPages->IOUnitPage2.u.fields.Header;
        *ppbPageData  =  pPages->IOUnitPage2.u.abPageData;
        *pcbPage      = sizeof(pPages->IOUnitPage2);
        break;
    case 3:
        *ppPageHeader = &pPages->IOUnitPage3.u.fields.Header;
        *ppbPageData  =  pPages->IOUnitPage3.u.abPageData;
        *pcbPage      = sizeof(pPages->IOUnitPage3);
        break;
    case 4:
        *ppPageHeader = &pPages->IOUnitPage4.u.fields.Header;
        *ppbPageData  =  pPages->IOUnitPage4.u.abPageData;
        *pcbPage      = sizeof(pPages->IOUnitPage4);
        break;
    default:
        rc = -1;
    }

    return rc;
}

static int mptsas_config_ioc_page(MPTSASState *pLsiLogic,
                         MptConfigurationPagesSupported *pPages,
                         uint8_t PageNumber,
                         MptConfigurationPageHeader **ppPageHeader,
                         uint8_t **ppbPageData, size_t *pcbPage)
{
    int rc = 0;

    switch (PageNumber) {
    case 0:
        *ppPageHeader = &pPages->IOCPage0.u.fields.Header;
        *ppbPageData  =  pPages->IOCPage0.u.abPageData;
        *pcbPage      = sizeof(pPages->IOCPage0);
        break;
    case 1:
        *ppPageHeader = &pPages->IOCPage1.u.fields.Header;
        *ppbPageData  =  pPages->IOCPage1.u.abPageData;
        *pcbPage      = sizeof(pPages->IOCPage1);
        break;
    case 2:
        *ppPageHeader = &pPages->IOCPage2.u.fields.Header;
        *ppbPageData  =  pPages->IOCPage2.u.abPageData;
        *pcbPage      = sizeof(pPages->IOCPage2);
        break;
    case 3:
        *ppPageHeader = &pPages->IOCPage3.u.fields.Header;
        *ppbPageData  =  pPages->IOCPage3.u.abPageData;
        *pcbPage      = sizeof(pPages->IOCPage3);
        break;
    case 4:
        *ppPageHeader = &pPages->IOCPage4.u.fields.Header;
        *ppbPageData  =  pPages->IOCPage4.u.abPageData;
        *pcbPage      = sizeof(pPages->IOCPage4);
        break;
    case 6:
        *ppPageHeader = &pPages->IOCPage6.u.fields.Header;
        *ppbPageData  =  pPages->IOCPage6.u.abPageData;
        *pcbPage      = sizeof(pPages->IOCPage6);
        break;
    default:
        rc = -1;
    }

    return rc;
}

static int mptsas_config_manufacturing_page(MPTSASState *pLsiLogic,
                               MptConfigurationPagesSupported *pPages,
                               uint8_t PageNumber,
                               MptConfigurationPageHeader **ppPageHeader,
                               uint8_t **ppbPageData, size_t *pcbPage)
{
    int rc = 0;

    switch (PageNumber) {
    case 0:
        *ppPageHeader = &pPages->ManufacturingPage0.u.fields.Header;
        *ppbPageData  =  pPages->ManufacturingPage0.u.abPageData;
        *pcbPage      = sizeof(pPages->ManufacturingPage0);
        break;
    case 1:
        *ppPageHeader = &pPages->ManufacturingPage1.u.fields.Header;
        *ppbPageData  =  pPages->ManufacturingPage1.u.abPageData;
        *pcbPage      = sizeof(pPages->ManufacturingPage1);
        break;
    case 2:
        *ppPageHeader = &pPages->ManufacturingPage2.u.fields.Header;
        *ppbPageData  =  pPages->ManufacturingPage2.u.abPageData;
        *pcbPage      = sizeof(pPages->ManufacturingPage2);
        break;
    case 3:
        *ppPageHeader = &pPages->ManufacturingPage3.u.fields.Header;
        *ppbPageData  =  pPages->ManufacturingPage3.u.abPageData;
        *pcbPage      = sizeof(pPages->ManufacturingPage3);
        break;
    case 4:
        *ppPageHeader = &pPages->ManufacturingPage4.u.fields.Header;
        *ppbPageData  =  pPages->ManufacturingPage4.u.abPageData;
        *pcbPage      = sizeof(pPages->ManufacturingPage4);
        break;
    case 5:
        *ppPageHeader = &pPages->ManufacturingPage5.u.fields.Header;
        *ppbPageData  =  pPages->ManufacturingPage5.u.abPageData;
        *pcbPage      = sizeof(pPages->ManufacturingPage5);
        break;
    case 6:
        *ppPageHeader = &pPages->ManufacturingPage6.u.fields.Header;
        *ppbPageData  =  pPages->ManufacturingPage6.u.abPageData;
        *pcbPage      = sizeof(pPages->ManufacturingPage6);
        break;
    case 7:
        *ppPageHeader = &pPages->ManufacturingPage7.u.fields.Header;
        *ppbPageData  =  pPages->ManufacturingPage7.u.abPageData;
        *pcbPage      = sizeof(pPages->ManufacturingPage7);
        break;
    case 8:
        *ppPageHeader = &pPages->ManufacturingPage8.u.fields.Header;
        *ppbPageData  =  pPages->ManufacturingPage8.u.abPageData;
        *pcbPage      = sizeof(pPages->ManufacturingPage8);
        break;
    case 9:
        *ppPageHeader = &pPages->ManufacturingPage9.u.fields.Header;
        *ppbPageData  =  pPages->ManufacturingPage9.u.abPageData;
        *pcbPage      = sizeof(pPages->ManufacturingPage9);
        break;
    case 10:
        *ppPageHeader = &pPages->ManufacturingPage10.u.fields.Header;
        *ppbPageData  =  pPages->ManufacturingPage10.u.abPageData;
        *pcbPage      = sizeof(pPages->ManufacturingPage10);
        break;
    default:
        rc = -1;
    }

    return rc;
}

static int mptsas_config_bios_page(MPTSASState *pLsiLogic,
                              MptConfigurationPagesSupported *pPages,
                              uint8_t PageNumber,
                              MptConfigurationPageHeader **ppPageHeader,
                              uint8_t **ppbPageData, size_t *pcbPage)
{
    int rc = 0;

    switch (PageNumber) {
    case 1:
        *ppPageHeader = &pPages->BIOSPage1.u.fields.Header;
        *ppbPageData  =  pPages->BIOSPage1.u.abPageData;
        *pcbPage      = sizeof(pPages->BIOSPage1);
        break;
    case 2:
        *ppPageHeader = &pPages->BIOSPage2.u.fields.Header;
        *ppbPageData  =  pPages->BIOSPage2.u.abPageData;
        *pcbPage      = sizeof(pPages->BIOSPage2);
        break;
    case 4:
        *ppPageHeader = &pPages->BIOSPage4.u.fields.Header;
        *ppbPageData  =  pPages->BIOSPage4.u.abPageData;
        *pcbPage      = sizeof(pPages->BIOSPage4);
        break;
    default:
        rc = -1;
    }

    return rc;
}

static int mptsas_config_sas_unit(MPTSASState *pLsiLogic,
                       MptConfigurationPagesSupported *pPages,
                       uint8_t PageNumber,
                       MptExtendedConfigurationPageHeader **ppPageHeader,
                       uint8_t **ppbPageData, size_t *pcbPage)
{
    int rc = 0;

    switch (PageNumber) {
    case 0:
        *ppPageHeader = &pPages->SASIOUnitPage0.u.fields.ExtHeader;
        *ppbPageData  = pPages->SASIOUnitPage0.u.abPageData;
        *pcbPage      = sizeof(pPages->SASIOUnitPage0);
        break;
    case 1:
        *ppPageHeader = &pPages->SASIOUnitPage1.u.fields.ExtHeader;
        *ppbPageData  = pPages->SASIOUnitPage1.u.abPageData;
        *pcbPage      = sizeof(pPages->SASIOUnitPage1);
        break;
    case 2:
        *ppPageHeader = &pPages->SASIOUnitPage2.u.fields.ExtHeader;
        *ppbPageData  =  pPages->SASIOUnitPage2.u.abPageData;
        *pcbPage      = sizeof(pPages->SASIOUnitPage2);
        break;
    case 3:
        *ppPageHeader = &pPages->SASIOUnitPage3.u.fields.ExtHeader;
        *ppbPageData  =  pPages->SASIOUnitPage3.u.abPageData;
        *pcbPage      = sizeof(pPages->SASIOUnitPage3);
        break;
    default:
        rc = -1;
    }

    return rc;
}

static int mptsas_config_sas_phy(MPTSASState *pLsiLogic,
                MptConfigurationPagesSupported *pPages,
                uint8_t PageNumber,
                uint32_t u32PageAddress,
                MptExtendedConfigurationPageHeader **ppPageHeader,
                uint8_t **ppbPageData, size_t *pcbPage)
{
    int rc = 0;
    MptConfigurationPageAddress PageAddress = { .PageAddress = u32PageAddress };
    uint8_t AddressForm =
        MPT_CONFIGURATION_PAGE_ADDRESS_GET_SAS_FORM(PageAddress);
    MptPHY *pPHYPages = NULL;

    if (AddressForm == 0) { /* PHY number */
        uint8_t PhyNumber = PageAddress.SASPHY.Form0.PhyNumber;

        if (PhyNumber >= MPTSAS_NUM_PORTS) {
            return -1;
        }

        pPHYPages = &pPages->aPHYs[PhyNumber];
    } else if (AddressForm == 1) { /* Index form */
        uint16_t Index = PageAddress.SASPHY.Form1.Index;

        if (Index >= MPTSAS_NUM_PORTS) {
            return -1;
        }

        pPHYPages = &pPages->aPHYs[Index];
    } else {
        rc = -1; /* Correct? */
    }

    if (pPHYPages) {
        switch (PageNumber) {
        case 0:
            *ppPageHeader = &pPHYPages->SASPHYPage0.u.fields.ExtHeader;
            *ppbPageData  = pPHYPages->SASPHYPage0.u.abPageData;
            *pcbPage      = sizeof(pPHYPages->SASPHYPage0);
            break;
        case 1:
            *ppPageHeader = &pPHYPages->SASPHYPage1.u.fields.ExtHeader;
            *ppbPageData  =  pPHYPages->SASPHYPage1.u.abPageData;
            *pcbPage      = sizeof(pPHYPages->SASPHYPage1);
            break;
        default:
            rc = -1;
        }
    } else {
        rc = -1;
    }

    return rc;
}

static int mptsas_config_sas_device(MPTSASState *pLsiLogic,
                   MptConfigurationPagesSupported *pPages,
                   uint8_t PageNumber,
                   uint32_t u32PageAddress,
                   MptExtendedConfigurationPageHeader **ppPageHeader,
                   uint8_t **ppbPageData, size_t *pcbPage)
{
    int rc = 0;
    MptConfigurationPageAddress PageAddress = { .PageAddress = u32PageAddress };
    uint8_t AddressForm =
        MPT_CONFIGURATION_PAGE_ADDRESS_GET_SAS_FORM(PageAddress);
    MptSASDevice *pSASDevice = NULL;

    if (AddressForm == 0) {
        uint16_t Handle = PageAddress.SASDevice.Form0And2.Handle;

        pSASDevice = pPages->pSASDeviceHead;

        /* Get the first device? */
        if (Handle != 0xffff) {
            /* No, search for the right one. */

            while (pSASDevice
                   && pSASDevice->SASDevicePage0.u.fields.DevHandle !=
                        Handle)
                pSASDevice = pSASDevice->pNext;

            if (pSASDevice) {
                pSASDevice = pSASDevice->pNext;
            }
        }
    } else if (AddressForm == 1) {
        uint8_t TargetID = PageAddress.SASDevice.Form1.TargetID;
        uint8_t Bus      = PageAddress.SASDevice.Form1.Bus;

        pSASDevice = pPages->pSASDeviceHead;

        while (pSASDevice
               && (pSASDevice->SASDevicePage0.u.fields.TargetID !=
                        TargetID
                   || pSASDevice->SASDevicePage0.u.fields.Bus != Bus))
            pSASDevice = pSASDevice->pNext;
    } else if (AddressForm == 2) {
        uint16_t Handle = PageAddress.SASDevice.Form0And2.Handle;

        pSASDevice = pPages->pSASDeviceHead;

        while (pSASDevice
               && pSASDevice->SASDevicePage0.u.fields.DevHandle !=
                Handle) {
            pSASDevice = pSASDevice->pNext;
        }
    }

    if (pSASDevice) {
        switch (PageNumber) {
        case 0:
            *ppPageHeader = &pSASDevice->SASDevicePage0.u.fields.ExtHeader;
            *ppbPageData  =  pSASDevice->SASDevicePage0.u.abPageData;
            *pcbPage      = sizeof(pSASDevice->SASDevicePage0);
            break;
        case 1:
            *ppPageHeader = &pSASDevice->SASDevicePage1.u.fields.ExtHeader;
            *ppbPageData  =  pSASDevice->SASDevicePage1.u.abPageData;
            *pcbPage      = sizeof(pSASDevice->SASDevicePage1);
            break;
        case 2:
            *ppPageHeader = &pSASDevice->SASDevicePage2.u.fields.ExtHeader;
            *ppbPageData  =  pSASDevice->SASDevicePage2.u.abPageData;
            *pcbPage      = sizeof(pSASDevice->SASDevicePage2);
            break;
        default:
            rc = -1;
        }
    } else {
        rc = -1;
    }

    return rc;
}

static int mptsas_config_page_get_extended(MPTSASState *pLsiLogic,
                MPIMsgConfig *pConfigurationReq,
                MptExtendedConfigurationPageHeader **ppPageHeader,
                uint8_t **ppbPageData, size_t *pcbPage)
{
    int rc = 0;

    switch (pConfigurationReq->ExtPageType) {
    case MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASIOUNIT:
    {
        rc = mptsas_config_sas_unit(pLsiLogic,
                                     pLsiLogic->config_pages,
                                     pConfigurationReq->PageNumber,
                                     ppPageHeader, ppbPageData, pcbPage);
        break;
    }
    case MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASPHYS:
    {
        rc = mptsas_config_sas_phy(pLsiLogic,
                                      pLsiLogic->config_pages,
                                      pConfigurationReq->PageNumber,
                                      pConfigurationReq->PageAddress,
                                      ppPageHeader, ppbPageData, pcbPage);
        break;
    }
    case MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASDEVICE:
    {
        rc = mptsas_config_sas_device(pLsiLogic,
                                     pLsiLogic->config_pages,
                                     pConfigurationReq->PageNumber,
                                     pConfigurationReq->PageAddress,
                                     ppPageHeader, ppbPageData, pcbPage);
        break;
    }
    case MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASEXPANDER:
        /* No expanders supported */
    case MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_ENCLOSURE:
        /* No enclosures supported */
    default:
        rc = -1;
    }

    return rc;
}


void mptsas_process_config(MPTSASState *s, MPIMsgConfig *req)
{
    MPIMsgConfigReply reply;

    int rc = -1;
    uint8_t                            *pbPageData     = NULL;
    MptConfigurationPageHeader         *pPageHeader    = NULL;
    MptExtendedConfigurationPageHeader *pExtPageHeader = NULL;
    size_t                              cbPage = 0;

    mptsas_fix_config_endianness(req);

    QEMU_BUILD_BUG_ON(sizeof(s->doorbell_msg) < sizeof(*req));
    QEMU_BUILD_BUG_ON(sizeof(s->doorbell_reply) < sizeof(reply));

    /* Copy common bits from the request into the reply. */
    reply.Action          = req->Action;
    reply.Function        = req->Function;
    reply.MsgContext = req->MsgContext;
    reply.MsgLength   = sizeof(reply) / 4;

    switch (MPT_CONFIGURATION_PAGE_TYPE_GET(req->PageType)) {
    case MPT_CONFIGURATION_PAGE_TYPE_IO_UNIT:
    {
        rc = mptsas_config_unit_page(s, s->config_pages,
                  req->PageNumber,
                  &pPageHeader, &pbPageData, &cbPage);
        break;
    }
    case MPT_CONFIGURATION_PAGE_TYPE_IOC:
    {
        /* Get the page data. */
        rc = mptsas_config_ioc_page(s, s->config_pages,
                  req->PageNumber,
                  &pPageHeader, &pbPageData, &cbPage);
        break;
    }
    case MPT_CONFIGURATION_PAGE_TYPE_MANUFACTURING:
    {
        rc = mptsas_config_manufacturing_page(s, s->config_pages,
                 req->PageNumber,
                 &pPageHeader, &pbPageData, &cbPage);
        break;
    }
    case MPT_CONFIGURATION_PAGE_TYPE_BIOS:
    {
        rc = mptsas_config_bios_page(s, s->config_pages,
                            req->PageNumber,
                            &pPageHeader, &pbPageData, &cbPage);
        break;
    }
    case MPT_CONFIGURATION_PAGE_TYPE_EXTENDED:
    {
        rc = mptsas_config_page_get_extended(s, req,
            &pExtPageHeader, &pbPageData, &cbPage);
        break;
    }
    default:
        break;
    }

    if (rc == -1) {
        reply.PageType    = req->PageType;
        reply.PageNumber  = req->PageNumber;
        reply.PageLength  = req->PageLength;
        reply.PageVersion = req->PageVersion;
        reply.IOCStatus  = MPI_IOCSTATUS_CONFIG_INVALID_PAGE;
        goto out;
    }

    if (MPT_CONFIGURATION_PAGE_TYPE_GET(req->PageType) ==
                MPT_CONFIGURATION_PAGE_TYPE_EXTENDED) {
        reply.PageType       = pExtPageHeader->PageType;
        reply.PageNumber     = pExtPageHeader->PageNumber;
        reply.PageVersion    = pExtPageHeader->PageVersion;
        reply.ExtPageType    = pExtPageHeader->ExtPageType;
        reply.ExtPageLength = pExtPageHeader->ExtPageLength;
    } else {
        reply.PageType    = pPageHeader->PageType;
        reply.PageNumber  = pPageHeader->PageNumber;
        reply.PageLength  = pPageHeader->PageLength;
        reply.PageVersion = pPageHeader->PageVersion;
    }

    /*
     * Don't use the scatter gather handling code as the configuration
     * request always have only one simple element.
     */
    switch (req->Action) {
    case MPI_CONFIG_ACTION_PAGE_DEFAULT:
        /* Nothing to do. We are always using the defaults. */
    case MPI_CONFIG_ACTION_PAGE_HEADER:
    {
        /* Already copied above nothing to do. */
        break;
    }
    case MPI_CONFIG_ACTION_PAGE_READ_NVRAM:
    case MPI_CONFIG_ACTION_PAGE_READ_CURRENT:
    case MPI_CONFIG_ACTION_PAGE_READ_DEFAULT:
    {
        uint32_t flags_and_length = req->PageBufferSGE.FlagsLength;
        uint32_t length = flags_and_length & MPI_SGE_LENGTH_MASK;
        if (length != 0) {
            uint64_t page_buffer_pa;
            if (flags_and_length & MPI_SGE_FLAGS_64_BIT_ADDRESSING) {
                page_buffer_pa = req->PageBufferSGE.u.Address64;
            } else {
                page_buffer_pa = req->PageBufferSGE.u.Address32;
            }

            cpu_physical_memory_write(page_buffer_pa, pbPageData, MIN(length,
                cbPage));
        }
        break;
    }
    case MPI_CONFIG_ACTION_PAGE_WRITE_CURRENT:
    case MPI_CONFIG_ACTION_PAGE_WRITE_NVRAM:
    {
        uint32_t flags_and_length = req->PageBufferSGE.FlagsLength;
        uint32_t length = flags_and_length & MPI_SGE_LENGTH_MASK;
        if (length != 0) {
            uint64_t page_buffer_pa;
            if (flags_and_length & MPI_SGE_FLAGS_64_BIT_ADDRESSING) {
                page_buffer_pa = req->PageBufferSGE.u.Address64;
            } else {
                page_buffer_pa = req->PageBufferSGE.u.Address32;
            }

            cpu_physical_memory_read(page_buffer_pa, pbPageData, MIN(length,
                cbPage));
        }
        break;
    }
    default:
        break;
    }

out:
    mptsas_fix_config_reply_endianness(&reply);
    mptsas_reply(s, (MPIDefaultReply *)&reply);
}


