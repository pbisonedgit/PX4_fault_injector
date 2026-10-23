# PX4 acknowledgement for one FAULT fault-management command.
# The target_system and target_component fields identify the external sender
# to which the MAVLink acknowledgement must be returned.

uint8 PROTOCOL_VERSION = 1

uint8 RESULT_ACCEPTED = 0
uint8 RESULT_REJECTED = 1
uint8 RESULT_DUPLICATE = 2
uint8 RESULT_NOT_FOUND = 3
uint8 RESULT_INTERNAL_ERROR = 4

uint8 REASON_NONE = 0
uint8 REASON_DISABLED = 1
uint8 REASON_ARMED_NOT_ALLOWED = 2
uint8 REASON_INVALID_PROTOCOL_VERSION = 3
uint8 REASON_INVALID_ACTION = 4
uint8 REASON_INVALID_TARGET = 5
uint8 REASON_INVALID_MODE = 6
uint8 REASON_INVALID_INSTANCE_MASK = 7
uint8 REASON_INVALID_AXIS_MASK = 8
uint8 REASON_INVALID_DURATION = 9
uint8 REASON_NO_FREE_SLOT = 10
uint8 REASON_UNKNOWN_FAULT_ID = 11
uint8 REASON_DUPLICATE_SEQUENCE = 12
uint8 REASON_INVALID_PARAMETER = 13
uint8 REASON_DUPLICATE_FAULT_ID = 14

uint8 STATE_EMPTY = 0
uint8 STATE_PENDING = 1
uint8 STATE_ACTIVE = 2
uint8 STATE_CLEARED = 3
uint8 STATE_EXPIRED = 4
uint8 STATE_REJECTED = 5

uint8 ORB_QUEUE_LENGTH = 8

uint64 timestamp                 # [us] PX4 time when the acknowledgement was generated

uint32 scenario_id               # Scenario identifier copied from the command
uint32 sequence                  # Command sequence copied from the command

uint16 fault_id                  # Fault identifier copied from the command

uint8 target_system              # External MAVLink system receiving the acknowledgement
uint8 target_component           # External MAVLink component receiving the acknowledgement
uint8 protocol_version           # FAULT fault protocol version used by PX4
uint8 result                     # RESULT_*
uint8 reason                     # REASON_*
uint8 state                      # STATE_* after command processing
