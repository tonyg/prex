#ifndef _PCI_H
#define _PCI_H

#define N_PCI_BASE_ADDRESS_REGISTERS 6

#define PCI_HEADER_TYPE_GENERAL	0x00
#define PCI_HEADER_TYPE_BRIDGE	0x01
#define PCI_HEADER_TYPE_CARDBUS	0x02

#define PCI_HEADER_TYPE_MULTI_FUNCTION_MASK	0x80
#define PCI_HEADER_TYPE_MASK			0x7f

typedef enum pci_class_code_t_ {
  PCI_CLASS_LEGACY = 0x00,	/* Device was built prior definition of the class code field */
  PCI_CLASS_STORAGE = 0x01,	/* Mass Storage Controller */
  PCI_CLASS_NETWORK = 0x02,	/* Network Controller */
  PCI_CLASS_DISPLAY = 0x03,	/* Display Controller */
  PCI_CLASS_MULTIMEDIA = 0x04,	/* Multimedia Controller */
  PCI_CLASS_MEMORY = 0x05,	/* Memory Controller */
  PCI_CLASS_BRIDGE = 0x06,	/* Bridge Device */
  PCI_CLASS_SIMPLE_COMM = 0x07,	/* Simple Communication Controllers */
  PCI_CLASS_BASE_SYSTEM_PERIPHERAL = 0x08,	/* Base System Peripherals */
  PCI_CLASS_INPUT = 0x09,	/* Input Devices */
  PCI_CLASS_DOCK = 0x0A,	/* Docking Stations */
  PCI_CLASS_PROCESSOR = 0x0B,	/* Processors */
  PCI_CLASS_SERIAL_BUS = 0x0C,	/* Serial Bus Controllers */
  PCI_CLASS_WIRELESS = 0x0D,	/* Wireless Controllers */
  PCI_CLASS_INTELLIGENT_IO = 0x0E,	/* Intelligent I/O Controllers */
  PCI_CLASS_SATELLITE_COMM = 0x0F,	/* Satellite Communication Controllers */
  PCI_CLASS_CRYPTO = 0x10,	/* Encryption/Decryption Controllers */
  PCI_CLASS_DASP = 0x11,	/* Data Acquisition and Signal Processing Controllers */
  /* others reserved */
  PCI_CLASS_MISC = 0xFF		/* Device does not fit any defined class.  */
} pci_class_code_t;

typedef enum pci_register_offset_t_ {
  PCI_REGISTER_VENDOR_ID = 0,
  PCI_REGISTER_DEVICE_ID = 2,
  PCI_REGISTER_COMMAND = 4,
  PCI_REGISTER_STATUS = 6,
  PCI_REGISTER_REVISION_ID = 8,
  PCI_REGISTER_PROG_IF = 9,
  PCI_REGISTER_SUBCLASS = 10,
  PCI_REGISTER_CLASS_CODE = 11,
  PCI_REGISTER_CACHE_LINE_SIZE = 12,
  PCI_REGISTER_LATENCY_TIMER = 13,
  PCI_REGISTER_HEADER_TYPE = 14,
  PCI_REGISTER_BIST = 15,

  PCI_REGISTER_BAR0 = 16,
  PCI_REGISTER_BAR1 = 20,
  PCI_REGISTER_BAR2 = 24,
  PCI_REGISTER_BAR3 = 28,
  PCI_REGISTER_BAR4 = 32,
  PCI_REGISTER_BAR5 = 36,

  PCI_REGISTER_CARDBUS_CIS_POINTER = 40,
  PCI_REGISTER_SUBSYSTEM_VENDOR_ID = 44,
  PCI_REGISTER_SUBSYSTEM_ID = 46,
  PCI_REGISTER_EXPANSION_ROM_BASE_ADDRESS = 48,
  PCI_REGISTER_CAPABILITIES_POINTER = 52,
  PCI_REGISTER_INTERRUPT_LINE = 60,
  PCI_REGISTER_INTERRUPT_PIN = 61,
  PCI_REGISTER_MIN_GRANT = 62,
  PCI_REGISTER_MAX_LATENCY = 63
} pci_register_offset_t;

struct pci_device {
  int bus;
  int slot;
  int function;

  uint16_t vendor_id;
  uint16_t device_id;
  uint8_t revision_id;
  uint8_t prog_if;
  uint8_t subclass;
  uint8_t class_code;
  uint8_t cache_line_size;
  uint8_t latency_timer;
  uint8_t header_type;
  uint8_t bist;

  uint16_t subsystem_vendor_id;
  uint16_t subsystem_id;
};

extern struct pci_device pci_devices[];
extern size_t pci_device_count; /* used slots - some may have vendor 0xffff meaning invalid */

extern uint32_t read_pci32(int bus, int dev, int fn, int offset);
extern uint16_t read_pci16(int bus, int dev, int fn, int offset);
extern uint8_t read_pci8(int bus, int dev, int fn, int offset);

extern void write_pci32(int bus, int dev, int fn, int offset, uint32_t val);
extern void write_pci16(int bus, int dev, int fn, int offset, uint16_t val);
extern void write_pci8(int bus, int dev, int fn, int offset, uint8_t val);

extern uint8_t read_pci_interrupt_line(int bus, int dev);
extern uint32_t read_pci_bar(int bus, int dev, int bar_number);

extern void write_pci_interrupt_line(int bus, int dev, int irqno);
extern void write_pci_bar(int bus, int dev, int bar_number, uint32_t val);

#endif
