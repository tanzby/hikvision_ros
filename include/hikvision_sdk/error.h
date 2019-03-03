#ifndef HIKVISION_ROS_ERROR_H
#define HIKVISION_ROS_ERROR_H
#include <string>
using namespace std;

/**
 * @brief Reference from "设备网络SDK指南（IPC）.pdf", For detailed information on the errors' code.
 *        Please go to ![](https://www.hikvision.com/cn/download_61.html)
 * @param code
 * @return
 */
std::string get_error_str(int code)
{
    switch (code)
    {
        case 0: return "NET_DVR_NOERROR";
        case 1: return "NET_DVR_PASSWORD_ERROR";
        case 2: return "NET_DVR_NOENOUGHPRI";
        case 3: return "NET_DVR_NOINIT";
        case 4: return "NET_DVR_CHANNEL_ERROR";
        case 5: return "NET_DVR_OVER_MAXLINK";
        case 6: return "NET_DVR_VERSIONNOMATCH";
        case 7: return "NET_DVR_NETWORK_FAIL_CONNECT";
        case 8: return "NET_DVR_NETWORK_SEND_ERROR";
        case 9: return "NET_DVR_NETWORK_RECV_ERROR";
        case 10: return "NET_DVR_NETWORK_RECV_TIMEOUT";
        case 11: return "NET_DVR_NETWORK_ERRORDATA";
        case 12: return "NET_DVR_ORDER_ERROR";
        case 13: return "NET_DVR_OPERNOPERMIT";
        case 14: return "NET_DVR_COMMANDTIMEOUT";
        case 15: return "NET_DVR_ERRORSERIALPORT";
        case 16: return "NET_DVR_ERRORALARMPORT";
        case 17: return "NET_DVR_PARAMETER_ERROR";
        case 18: return "NET_DVR_CHAN_EXCEPTION";
        case 19: return "NET_DVR_NODISK";
        case 20: return "NET_DVR_ERRORDISKNUM";
        case 21: return "NET_DVR_DISK_FULL";
        case 22: return "NET_DVR_DISK_ERROR";
        case 23: return "NET_DVR_NOSUPPORT";
        case 24: return "NET_DVR_BUSY";
        case 25: return "NET_DVR_MODIFY_FAIL";
        case 26: return "NET_DVR_PASSWORD_FORMAT_ERROR";
        case 27: return "NET_DVR_DISK_FORMATING";
        case 28: return "NET_DVR_DVRNORESOURCE";
        case 29: return "NET_DVR_DVROPRATEFAILED";
        case 30: return "NET_DVR_OPENHOSTSOUND_FAIL";
        case 31: return "NET_DVR_DVRVOICEOPENED";
        case 32: return "NET_DVR_TIMEINPUTERROR";
        case 33: return "NET_DVR_NOSPECFILE";
        case 34: return "NET_DVR_CREATEFILE_ERROR";
        case 35: return "NET_DVR_FILEOPENFAIL";
        case 36: return "NET_DVR_OPERNOTFINISH";
        case 37: return "NET_DVR_GETPLAYTIMEFAIL";
        case 38: return "NET_DVR_PLAYFAIL";
        case 39: return "NET_DVR_FILEFORMAT_ERROR";
        case 40: return "NET_DVR_DIR_ERROR";
        case 41: return "NET_DVR_ALLOC_RESOURCE_ERROR";
        case 42: return "NET_DVR_AUDIO_MODE_ERROR";
        case 43: return "NET_DVR_NOENOUGH_BUF";
        case 44: return "NET_DVR_CREATESOCKET_ERROR";
        case 45: return "NET_DVR_SETSOCKET_ERROR";
        case 46: return "NET_DVR_MAX_NUM";
        case 47: return "NET_DVR_USERNOTEXIST";
        case 48: return "NET_DVR_WRITEFLASHERROR";
        case 49: return "NET_DVR_UPGRADEFAIL";
        case 50: return "NET_DVR_CARDHAVEINIT";
        case 51: return "NET_DVR_PLAYERFAILED";
        case 52: return "NET_DVR_MAX_USERNUM";
        case 53: return "NET_DVR_GETLOCALIPANDMACFAIL";
        case 54: return "NET_DVR_NOENCODEING";
        case 55: return "NET_DVR_IPMISMATCH";
        case 56: return "NET_DVR_MACMISMATCH";
        case 57: return "NET_DVR_UPGRADELANGMISMATCH";
        case 58: return "NET_DVR_MAX_PLAYERPORT";
        case 59: return "NET_DVR_NOSPACEBACKUP";
        case 60: return "NET_DVR_NODEVICEBACKUP";
        case 61: return "NET_DVR_PICTURE_BITS_ERROR";
        case 62: return "NET_DVR_PICTURE_DIMENSION_ERROR";
        case 63: return "NET_DVR_PICTURE_SIZ_ERROR";
        case 64: return "NET_DVR_LOADPLAYERSDKFAILED";
        case 65: return "NET_DVR_LOADPLAYERSDKPROC_ERROR";
        case 66: return "NET_DVR_LOADDSSDKFAILED";
        case 67: return "NET_DVR_LOADDSSDKPROC_ERROR";
        case 68: return "NET_DVR_DSSDK_ERROR";
        case 69: return "NET_DVR_VOICEMONOPOLIZE";
        case 70: return "NET_DVR_JOINMULTICASTFAILED";
        case 71: return "NET_DVR_CREATEDIR_ERROR";
        case 72: return "NET_DVR_BINDSOCKET_ERROR";
        case 73: return "NET_DVR_SOCKETCLOSE_ERROR";
        case 74: return "NET_DVR_USERID_ISUSING";
        case 75: return "NET_DVR_SOCKETLISTEN_ERROR";
        case 76: return "NET_DVR_PROGRAM_EXCEPTION";
        case 77: return "NET_DVR_WRITEFILE_FAILED";
        case 78: return "NET_DVR_FORMAT_READONLY";
        case 79: return "NET_DVR_WITHSAMEUSERNAME";
        case 80: return "NET_DVR_DEVICETYPE_ERROR";
        case 81: return "NET_DVR_LANGUAGE_ERROR";
        case 82: return "NET_DVR_PARAVERSION_ERROR";
        case 83: return "NET_DVR_IPCHAN_NOTALIVE";
        case 84: return "NET_DVR_RTSP_SDK_ERROR";
        case 85: return "NET_DVR_CONVERT_SDK_ERROR";
        case 86: return "NET_DVR_IPC_COUNT_OVERFLOW";
        case 87: return "NET_DVR_MAX_ADD_NUM";
        case 88: return "NET_DVR_PARAMMODE_ERROR";
        case 89: return "NET_DVR_CODESPITTER_OFFLINE";
        case 90: return "NET_DVR_BACKUP_COPYING";
        case 91: return "NET_DVR_CHAN_NOTSUPPORT";
        case 92: return "NET_DVR_CALLINEINVALID";
        case 93: return "NET_DVR_CALCANCELCONFLICT";
        case 94: return "NET_DVR_CALPOINTOUTRANGE";
        case 95: return "NET_DVR_FILTERRECTINVALID";
        case 96: return "NET_DVR_DDNS_DEVOFFLINE";
        case 97: return "NET_DVR_DDNS_INTER_ERROR";
        case 98: return "NET_DVR_FUNCTION_NOT_SUPPORT_OS";
        case 99: return "NET_DVR_DEC_CHAN_REBIND";
        case 100: return "NET_DVR_INTERCOM_SDK_ERROR";
        case 101: return "NET_DVR_NO_CURRENT_UPDATEFILE";
        case 102: return "NET_DVR_USER_NOT_SUCC_LOGIN";
        case 103: return "NET_DVR_USE_LOG_SWITCH_FILE";
        case 104: return "NET_DVR_POOL_PORT_EXHAUST";
        case 105: return "NET_DVR_PACKET_TYPE_NOT_SUPPORT";
        case 106: return "NET_DVR_IPPARA_IPID_ERROR";
        case 107: return "NET_DVR_LOAD_HCPREVIEW_SDK_ERROR";
        case 108: return "NET_DVR_LOAD_HCVOICETALK_SDK_ERROR";
        case 109: return "NET_DVR_LOAD_HCALARM_SDK_ERROR";
        case 110: return "NET_DVR_LOAD_HCPLAYBACK_SDK_ERROR";
        case 111: return "NET_DVR_LOAD_HCDISPLAY_SDK_ERROR";
        case 112: return "NET_DVR_LOAD_HCINDUSTRY_SDK_ERROR";
        case 113: return "NET_DVR_LOAD_HCGENERALCFGMGR_SDK_ERROR";
        case 114: return "NET_DVR_LOAD_HCCOREDEVCFG_SDK_ERROR";
        case 121: return "NET_DVR_CORE_VER_MISMATCH";
        case 122: return "NET_DVR_CORE_VER_MISMATCH_HCPREVIEW";
        case 123: return "NET_DVR_CORE_VER_MISMATCH_HCVOICETALK";
        case 124: return "NET_DVR_CORE_VER_MISMATCH_HCALARM";
        case 125: return "NET_DVR_CORE_VER_MISMATCH_HCPLAYBACK";
        case 126: return "NET_DVR_CORE_VER_MISMATCH_HCDISPLAY";
        case 127: return "NET_DVR_CORE_VER_MISMATCH_HCINDUSTRY";
        case 128: return "NET_DVR_CORE_VER_MISMATCH_HCGENERALCFGMGR";
        case 136: return "NET_DVR_COM_VER_MISMATCH_HCPREVIEW";
        case 137: return "NET_DVR_COM_VER_MISMATCH_HCVOICETALK";
        case 138: return "NET_DVR_COM_VER_MISMATCH_HCALARM";
        case 139: return "NET_DVR_COM_VER_MISMATCH_HCPLAYBACK";
        case 140: return "NET_DVR_COM_VER_MISMATCH_HCDISPLAY";
        case 141: return "NET_DVR_COM_VER_MISMATCH_HCINDUSTRY";
        case 142: return "NET_DVR_COM_VER_MISMATCH_HCGENERALCFGMGR";
        case 150: return "NET_DVR_ALIAS_DUPLICATE";
        case 152: return "NET_DVR_USERNAME_NOT_EXIST";
        case 153: return "NET_ERR_USERNAME_LOCKED";
        case 154: return "NET_DVR_INVALID_USERID";
        case 155: return "NET_DVR_LOW_LOGIN_VERSION";
        case 156: return "NET_DVR_LOAD_LIBEAY32_DLL_ERROR";
        case 157: return "NET_DVR_LOAD_SSLEAY32_DLL_ERROR";
        case 158: return "NET_ERR_LOAD_LIBICONV";
        case 165: return "NET_DVR_TEST_SERVER_FAIL_CONNECT";
        case 166: return "NET_DVR_NAS_SERVER_INVALID_DIR";
        case 167: return "NET_DVR_NAS_SERVER_NOENOUGH_PRI";
        case 168: return "NET_DVR_EMAIL_SERVER_NOT_CONFIG_DNS";
        case 169: return "NET_DVR_EMAIL_SERVER_NOT_CONFIG_GATEWAY";
        case 170: return "NET_DVR_TEST_SERVER_PASSWORD_ERROR";
        case 171: return "NET_DVR_EMAIL_SERVER_CONNECT_EXCEPTION_WITH_SMTP";
        case 172: return "NET_DVR_FTP_SERVER_FAIL_CREATE_DIR";
        case 173: return "NET_DVR_FTP_SERVER_NO_WRITE_PIR";
        case 174: return "NET_DVR_IP_CONFLICT";
        case 175: return "NET_DVR_INSUFFICIENT_STORAGEPOOL_SPACE";
        case 176: return "NET_DVR_STORAGEPOOL_INVALID";
        case 177: return "NET_DVR_EFFECTIVENESS_REBOOT";
        case 178: return "NET_ERR_ANR_ARMING_EXIST";
        case 179: return "NET_ERR_UPLOADLINK_EXIST";
        case 180: return "NET_ERR_INCORRECT_FILE_FORMAT";
        case 181: return "NET_ERR_INCORRECT_FILE_CONTENT";
        case 182: return "NET_ERR_MAX_HRUDP_LINK";


        case 500: return "NET_PLAYM4_NOERROR";
        case 501: return "NET_PLAYM4_PARA_OVER";
        case 502: return "NET_PLAYM4_ORDER_ERROR";
        case 503: return "NET_PLAYM4_TIMER_ERROR";
        case 504: return "NET_PLAYM4_DEC_VIDEO_ERROR";
        case 505: return "NET_PLAYM4_DEC_AUDIO_ERROR";
        case 506: return "NET_PLAYM4_ALLOC_MEMORY_ERROR";
        case 507: return "NET_PLAYM4_OPEN_FILE_ERROR";
        case 508: return "NET_PLAYM4_CREATE_OBJ_ERROR";
        case 509: return "NET_PLAYM4_CREATE_DDRAW_ERROR";
        case 510: return "NET_PLAYM4_CREATE_OFFSCREEN_ERROR";
        case 511: return "NET_PLAYM4_BUF_OVER";
        case 512: return "NET_PLAYM4_CREATE_SOUND_ERROR";
        case 513: return "NET_PLAYM4_SET_VOLUME_ERROR";
        case 514: return "NET_PLAYM4_SUPPORT_FILE_ONLY";
        case 515: return "NET_PLAYM4_SUPPORT_STREAM_ONLY";
        case 516: return "NET_PLAYM4_SYS_NOT_SUPPORT";
        case 517: return "NET_PLAYM4_FILEHEADER_UNKNOWN";
        case 518: return "NET_PLAYM4_VERSION_INCORRECT";
        case 519: return "NET_PALYM4_INIT_DECODER_ERROR";
        case 520: return "NET_PLAYM4_CHECK_FILE_ERROR";
        case 521: return "NET_PLAYM4_INIT_TIMER_ERROR";
        case 522: return "NET_PLAYM4_BLT_ERROR";
        case 523: return "NET_PLAYM4_UPDATE_ERROR";
        case 524: return "NET_PLAYM4_OPEN_FILE_ERROR_MULTI";
        case 525: return "NET_PLAYM4_OPEN_FILE_ERROR_VIDEO";
        case 526: return "NET_PLAYM4_JPEG_COMPRESS_ERROR";
        case 527: return "NET_PLAYM4_EXTRACT_NOT_SUPPORT";
        case 528: return "NET_PLAYM4_EXTRACT_DATA_ERROR";

        default:
            return std::to_string(code);
    }
}

#endif //HIKVISION_ROS_ERROR_H
