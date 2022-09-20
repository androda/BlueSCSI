#ifndef __SCSI_SENSE_H__
#define __SCSI_SENSE_H__

#define SCSI_SENSE_NO_SENSE         0
#define SCSI_SENSE_RECOVERED_ERROR  0x1
#define SCSI_SENSE_NOT_READY        0x2
#define SCSI_SENSE_MEDUIM_ERROR     0x3
#define SCSI_SENSE_HARDWARE_ERROR   0x4
#define SCSI_SENSE_ILLEGAL_REQUEST  0x5
#define SCSI_SENSE_UNIT_ATTENTION   0x6
#define SCSI_SENSE_DATA_PROTECT     0x7
#define SCSI_SENSE_BLANK_CHECK      0x8
#define SCSI_SENSE_VENDOR_SPECIFIC  0x9
#define SCSI_SENSE_COPY_ABORTED     0xa
#define SCSI_SENSE_ABORTED_COMMAND  0xb
#define SCSI_SENSE_EQUAL            0xc
#define SCSI_SENSE_VOLUME_OVERFLOW  0xd
#define SCSI_SENSE_MISCOMPARE       0xe
#define SCSI_SENSE_RESERVED         0xf


#define SCSI_ASC_INVALID_OPERATION_CODE                         0x2000
#define SCSI_ASC_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE             0x2100
#define SCSI_ASC_INVALID_FIELD_IN_CDB                           0x2400
#define SCSI_ASC_LOGICAL_UNIT_NOT_SUPPORTED                     0x2500
#define SCSI_ASC_INVALID_FIELD_PARAMETER_LIST                   0x2600
#define SCSI_ASC_WRITE_PROTECTED                                0x2700
#define SCSI_ASC_CANNOT_READ_MEDIUM_UNKNOWN_FORMAT              0x3001
#define SCSI_ASC_CANNOT_READ_MEDIUM_INCOMPATIBLE_FORMAT         0x3002
#define SCSI_ASC_SAVING_PARAMETERS_NOT_SUPPORTED                0x3900
#define SCSI_ASC_MEDIUM_NOT_PRESENT                             0x3A00
#define SCSI_ASC_LUN_NOT_READY_MANUAL_INTERVENTION_REQUIRED     0x0403


// SCSI mode page codes
#define SCSI_SENSE_MODE_VENDOR                      0x00
#define SCSI_SENSE_MODE_READ_WRITE_ERROR_RECOVERY   0x01
#define SCSI_SENSE_MODE_DISCONNECT_RECONNECT        0x02
#define SCSI_SENSE_MODE_FORMAT_DEVICE               0x03
#define SCSI_SENSE_MODE_DISK_GEOMETRY               0x04
#define SCSI_SENSE_MODE_FLEXABLE_GEOMETRY           0x05
#define SCSI_SENSE_MODE_CACHING                     0x08
#define SCSI_SENSE_MODE_CDROM                       0x0D
#define SCSI_SENSE_MODE_CDROM_AUDIO_CONTROL         0x0E
#define SCSI_SENSE_MODE_VENDOR_APPLE                0x30

#define SCSI_SENSE_MODE_ALL                         0x3F


#endif