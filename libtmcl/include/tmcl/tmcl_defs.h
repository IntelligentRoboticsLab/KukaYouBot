#ifndef TMCL_PROTO_H
#define TMCL_PROTO_H

enum tmcl_status_t {
	TMCL_TS_INVAL_CHKSUM = 1,
	TMCL_TS_INVAL_CMD = 2,
	TMCL_TS_INVAL_TYPE = 3,
	TMCL_TS_INVAL_VALUE = 4,
	TMCL_TS_CONFIG_LOCKED = 5,
	TMCL_TS_CMD_UNAVAIL = 6,
	TMCL_TS_EXEC_OK = 100,
	TMCL_TS_LOAD_OK = 101,
};

enum tmcl_cmd_id_t {
	TMCL_CI_ROR = 1,
	TMCL_CI_ROL = 2,
	TMCL_CI_MST = 3,
	TMCL_CI_MVP = 4,
	TMCL_CI_SAP = 5,
	TMCL_CI_GAP = 6,
	TMCL_CI_STAP = 7,
	TMCL_CI_RSAP = 8,
	TMCL_CI_SGP = 9,
	TMCL_CI_GGP = 10,
	TMCL_CI_STGP = 11,
	TMCL_CI_RSGP = 12,
	TMCL_CI_RFS = 13,
	TMCL_CI_SIO = 14,
	TMCL_CI_GIO = 15,
	TMCL_CI_CALC = 19,
	TMCL_CI_COMP = 20,
	TMCL_CI_JC = 21,
	TMCL_CI_JA = 22,
	TMCL_CI_CSUB = 23,
	TMCL_CI_RSUB = 24,
	TMCL_CI_EI = 25,
	TMCL_CI_DI = 26,
	TMCL_CI_WAIT = 27,
	TMCL_CI_STOP = 28,
	TMCL_CI_SCO = 30,
	TMCL_CI_GCO = 31,
	TMCL_CI_CCO = 32,
	TMCL_CI_CALCX = 33,
	TMCL_CI_AAP = 34,
	TMCL_CI_AGP = 35,
	TMCL_CI_CLE = 36,
	TMCL_CI_VECT = 37,
	TMCL_CI_RETI = 38,
	TMCL_CI_ACO = 39,
	TMCL_CI_UF0 = 64,
	TMCL_CI_UF1 = 65,
	TMCL_CI_UF2 = 66,
	TMCL_CI_UF3 = 67,
	TMCL_CI_UF4 = 68,
	TMCL_CI_UF5 = 69,
	TMCL_CI_UF6 = 70,
	TMCL_CI_UF7 = 71,
	TMCL_CI_CTL_STOP = 128,
	TMCL_CI_CTL_RUN = 129,
	TMCL_CI_CTL_STEP = 130,
	TMCL_CI_CTL_RESET = 131,
	TMCL_CI_CTL_INIT_DLM = 132,
	TMCL_CI_CTL_QUIT_DLM = 133,
	TMCL_CI_CTL_READ_MEM = 134,
	TMCL_CI_CTL_STATUS = 135,
	TMCL_CI_CTL_FW_VER = 136,
	TMCL_CI_CTL_FACTORY = 137,
	TMCL_CI_CTL_RTPRE = 138,
	TMCL_CI_CTL_ASCII = 139,
};

struct tmcl_cmd_t {
	uint8_t mod_addr;
	uint8_t cmd_id;
	uint8_t type_id;
	uint8_t motor_id;
	uint32_t value;
	uint8_t chksum;
};

struct tmcl_cmd_reply_t {
	uint8_t reply_addr;
	uint8_t mod_addr;
	uint8_t status;
	uint8_t cmd_id;
	uint32_t value;
	uint8_t chksum;
};

#endif /* TMCL_PROTO_H */

