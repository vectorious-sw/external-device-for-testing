/*
 Generated by xml2c from external xml file. 
*/

struct _Description_c {
    char TEXT[4];
    };
struct _Value {
    uint32_t VALUE;
    };
struct _VoltageHysteresisWidth {
    struct _Description_c Description_c;
    uint32_t N_Description_c;
    struct _Value Value;
    uint32_t N_Value;
    };
struct _Infrastructure {
    struct _VoltageHysteresisWidth VoltageHysteresisWidth;
    uint32_t N_VoltageHysteresisWidth;
    };
struct _VbattUpperThreshold {
    struct _Description_c Description_c;
    uint32_t N_Description_c;
    struct _Value Value;
    uint32_t N_Value;
    };
struct _VbattLowerThreshold {
    struct _Description_c Description_c;
    uint32_t N_Description_c;
    struct _Value Value;
    uint32_t N_Value;
    };
struct _PowerManagement {
    struct _VbattUpperThreshold VbattUpperThreshold;
    uint32_t N_VbattUpperThreshold;
    struct _VbattLowerThreshold VbattLowerThreshold;
    uint32_t N_VbattLowerThreshold;
    };
struct _crc {
    uint32_t VALUE;
    };

/* top struct definition */
struct _ConfigPrefs {
    struct _Infrastructure Infrastructure;
    uint32_t N_Infrastructure;
    struct _PowerManagement PowerManagement;
    uint32_t N_PowerManagement;
    struct _crc crc;
    uint32_t N_crc;
    };

/* struct implementation */
#ifdef BURN_CONFIG
const static struct _ConfigPrefs ConfigPrefs = {
    .Infrastructure = {
        .VoltageHysteresisWidth = {
            .Description_c = {
                // .TEXT = "The hysteresis width applicable for all the voltage measurements in mV",
                   .TEXT = "null",
                },
            .N_Description_c = 1,
            .Value = {
                .VALUE = 100,
                },
            .N_Value = 1,
            },
        .N_VoltageHysteresisWidth = 1,
        },
    .N_Infrastructure = 1,
    .PowerManagement = {
        .VbattUpperThreshold = {
            .Description_c = {
                // .TEXT = "Battery measurement upper threshold. Battery voltage above this threshold shall indicate the batter is full. 1mV unit",
                   .TEXT = "null",
                },
            .N_Description_c = 1,
            .Value = {
                .VALUE = 3000,
                },
            .N_Value = 1,
            },
        .N_VbattUpperThreshold = 1,
        .VbattLowerThreshold = {
            .Description_c = {
                // .TEXT = "Battery measurement lower threshold. Battery voltage bellow this  threshold shall indicate the batter is low and required charging. 1mV unit",
                   .TEXT = "null",
                },
            .N_Description_c = 1,
            .Value = {
                .VALUE = 2000,
                },
            .N_Value = 1,
            },
        .N_VbattLowerThreshold = 1,
        },
    .N_PowerManagement = 1,
    .crc = {
        .VALUE = 0,
        },
    .N_crc = 1,
    };
#endif
