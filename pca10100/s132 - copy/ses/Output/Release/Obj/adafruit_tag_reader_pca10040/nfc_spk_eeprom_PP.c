# 0 "C:\\Users\\haris\\Downloads\\DeviceDownload\\nRF5_SDK_17.1.0_ddde560\\nRF5_SDK_17.1.0_ddde560\\examples\\My Project\\adafruit_tag_reader\\nfc_spk_eeprom\\nfc_spk_eeprom.c"
# 0 "<built-in>"
# 0 "<command-line>"
# 1 "C:\\Users\\haris\\Downloads\\DeviceDownload\\nRF5_SDK_17.1.0_ddde560\\nRF5_SDK_17.1.0_ddde560\\examples\\My Project\\adafruit_tag_reader\\nfc_spk_eeprom\\nfc_spk_eeprom.c"
# 1 "C:\\Users\\haris\\Downloads\\DeviceDownload\\nRF5_SDK_17.1.0_ddde560\\nRF5_SDK_17.1.0_ddde560\\examples\\My Project\\adafruit_tag_reader\\nfc_spk_eeprom\\nfc_spk_eeprom.h" 1



# 1 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/stdint.h" 1 3 4
# 20 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/stdint.h" 3 4
# 1 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL.h" 1 3 4
# 17 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL.h" 3 4
# 1 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL_ConfDefaults.h" 1 3 4
# 35 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL_ConfDefaults.h" 3 4
# 1 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL_Conf.h" 1 3 4
# 25 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL_Conf.h" 3 4
# 1 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL_Arm_Conf.h" 1 3 4
# 483 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL_Arm_Conf.h" 3 4
  
# 483 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL_Arm_Conf.h" 3 4
 static __inline__ __attribute__((__always_inline__)) int __SEGGER_RTL_CLZ_U32_safe_inline(unsigned x) {
    return x == 0 ? 32 : __builtin_clz(x);
  }
# 681 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL_Arm_Conf.h" 3 4
static __inline__ __attribute__((__always_inline__)) void __SEGGER_RTL_WR_PARTIAL_WORD_func(char *addr, unsigned w, int n) {
  switch (n) {
  default: addr[3] = (-1) > 0 ? w : w >> 24;
  case 3: addr[2] = (-1) > 0 ? w >> 8 : w >> 16;
  case 2: addr[1] = (-1) > 0 ? w >> 16 : w >> 8;
  case 1: addr[0] = (-1) > 0 ? w >> 24 : w;
  case 0: ;
  }
}

static __inline__ __attribute__((__always_inline__)) void __SEGGER_RTL_WR_WORD_func(char *addr, unsigned w) {
  __SEGGER_RTL_WR_PARTIAL_WORD_func(addr, w, 4);
}

static __inline__ __attribute__((__always_inline__)) unsigned __SEGGER_RTL_RD_WORD_func(const void *addr) {
  const unsigned char *pAddr = (const unsigned char *)addr;

  return pAddr[0] * ((-1) > 0 ? 0x1000000u : 0x1u) +
         pAddr[1] * ((-1) > 0 ? 0x10000u : 0x100u) +
         pAddr[2] * ((-1) > 0 ? 0x100u : 0x10000u) +
         pAddr[3] * ((-1) > 0 ? 0x1u : 0x1000000u);
}

static __inline__ __attribute__((__always_inline__)) unsigned __SEGGER_RTL_BYTE_PATTERN_func(unsigned x) {
  return x * 0x01010101uL;
}

static __inline__ __attribute__((__always_inline__)) unsigned __SEGGER_RTL_FILL_HEAD_func(const void *pOrigin, unsigned Word, unsigned Standin) {
  unsigned Mask;
  unsigned Fill;

  (void)pOrigin;

  Fill = __SEGGER_RTL_BYTE_PATTERN_func(Standin);
# 725 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL_Arm_Conf.h" 3 4
  Mask = 0xFFFFFFFFu;
  Mask <<= 8 * 0;

  return (Word & Mask) | (Fill & ~Mask);


}

static __inline__ __attribute__((__always_inline__)) unsigned __SEGGER_RTL_FILL_TAIL_func(unsigned n, unsigned Word, unsigned Standin) {
  unsigned Mask;
  unsigned Fill;
# 750 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL_Arm_Conf.h" 3 4
  if (n >= 4) {
    return Word;
  } else {
    Fill = __SEGGER_RTL_BYTE_PATTERN_func(Standin);
    Mask = 0xFFFFFFFFu << (8 * n);

    return (Fill & Mask) | (Word & ~Mask);
  }


}

static __inline__ __attribute__((__always_inline__)) unsigned __SEGGER_RTL_ZBYTE_CHECK_func(unsigned x) {



  return __builtin_arm_uqsub8(0x01010101, x);






}

static __inline__ __attribute__((__always_inline__)) unsigned __SEGGER_RTL_DIFF_INDEX_func(unsigned x, unsigned y) {







  return __SEGGER_RTL_CLZ_U32_safe_inline(__builtin_bswap32(x^y)) / 8;
# 806 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL_Arm_Conf.h" 3 4
}

static __inline__ __attribute__((__always_inline__)) unsigned __SEGGER_RTL_DIFF_BYTE_func(unsigned x, int Index) {



  return (x >> (8*Index)) & 0xFF;

}

static __inline__ __attribute__((__always_inline__)) unsigned __SEGGER_RTL_ZBYTE_INDEX_func(unsigned x) {
# 828 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL_Arm_Conf.h" 3 4
  return __SEGGER_RTL_CLZ_U32_safe_inline(__builtin_bswap32(__SEGGER_RTL_ZBYTE_CHECK_func(x))) / 8;
# 859 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL_Arm_Conf.h" 3 4
}

static __inline__ __attribute__((__always_inline__)) _Bool __SEGGER_RTL_atomic_is_lock_free(unsigned size, const volatile void *ptr) {

  switch (size) {
  case 1: return 1;
  case 2: return ((unsigned)ptr & 1) == 0;
  case 4: return ((unsigned)ptr & 3) == 0;
  default: return 0;
  }
}
# 880 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL_Arm_Conf.h" 3 4
extern const int __aeabi_SIGABRT;
extern const int __aeabi_SIGINT;
extern const int __aeabi_SIGILL;
extern const int __aeabi_SIGFPE;
extern const int __aeabi_SIGSEGV;
extern const int __aeabi_SIGTERM;

void __aeabi_SIG_DFL(int sig);
void __aeabi_SIG_IGN(int sig);
void __aeabi_SIG_ERR(int sig);
# 26 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL_Conf.h" 2 3 4
# 36 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL_ConfDefaults.h" 2 3 4
# 18 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL.h" 2 3 4
# 38 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL.h" 3 4
  typedef struct __SEGGER_RTL_FILE_impl __SEGGER_RTL_FILE;




typedef struct __SEGGER_RTL_locale_data_s __SEGGER_RTL_locale_t;
typedef struct __SEGGER_RTL_locale_codeset_s __SEGGER_RTL_locale_codeset_t;

typedef struct {



  const char * decimal_point;
  const char * thousands_sep;
  const char * grouping;

  const char * int_curr_symbol;
  const char * currency_symbol;
  const char * mon_decimal_point;
  const char * mon_thousands_sep;
  const char * mon_grouping;
  const char * positive_sign;
  const char * negative_sign;

  char int_frac_digits;
  char frac_digits;
  char p_cs_precedes;
  char p_sep_by_space;
  char n_cs_precedes;
  char n_sep_by_space;
  char p_sign_posn;
  char n_sign_posn;
  char int_p_cs_precedes;
  char int_n_cs_precedes;
  char int_p_sep_by_space;
  char int_n_sep_by_space;
  char int_p_sign_posn;
  char int_n_sign_posn;




  const char *day_names;
  const char *abbrev_day_names;
  const char *month_names;
  const char *abbrev_month_names;
  const char *am_pm_indicator;
  const char *date_format;
  const char *time_format;
  const char *date_time_format;
} __SEGGER_RTL_locale_data_t;

struct __SEGGER_RTL_locale_data_s {
  const char * name;
  const __SEGGER_RTL_locale_data_t * data;
  const __SEGGER_RTL_locale_codeset_t * codeset;
};

struct __SEGGER_RTL_POSIX_locale_s {
  const __SEGGER_RTL_locale_t *__category[5];
};

struct timeval;
# 109 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL.h" 3 4
extern struct __SEGGER_RTL_POSIX_locale_s __SEGGER_RTL_global_locale;
extern struct __SEGGER_RTL_POSIX_locale_s * __SEGGER_RTL_locale_ptr;
extern char * __SEGGER_RTL_locale_name_buffer;
# 120 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL.h" 3 4
extern const __SEGGER_RTL_locale_t __SEGGER_RTL_c_locale;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_iso8859_1;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_iso8859_2;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_iso8859_3;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_iso8859_4;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_iso8859_5;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_iso8859_6;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_iso8859_7;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_iso8859_8;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_iso8859_9;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_iso8859_10;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_iso8859_11;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_iso8859_13;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_iso8859_14;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_iso8859_15;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_iso8859_16;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_cp1250;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_cp1251;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_cp1252;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_cp1253;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_cp1254;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_cp1255;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_cp1256;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_cp1257;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_cp1258;

extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_c_locale_data;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_aa_DJ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_aa_ER_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_aa_ET_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_af_NA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_af_ZA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ak_GH_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_am_ET_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ar_AE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ar_BH_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ar_DZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ar_EG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ar_IQ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ar_JO_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ar_KW_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ar_LB_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ar_LY_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ar_MA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ar_OM_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ar_QA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ar_SA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ar_SD_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ar_SY_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ar_TN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ar_YE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_as_IN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_asa_TZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_az_Arab_IR_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_az_AZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_az_Cyrl_AZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_az_IR_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_az_Latn_AZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_be_BY_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_bem_ZM_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_bez_TZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_bg_BG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_bm_ML_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_bn_BD_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_bn_IN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_bo_CN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_bo_IN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_br_FR_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_brx_IN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_bs_BA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_byn_ER_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ca_ES_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_cch_NG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_cgg_UG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_chr_US_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_cs_CZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_cy_GB_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_da_DK_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_dav_KE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_de_AT_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_de_BE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_de_CH_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_de_DE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_de_LI_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_de_LU_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_dv_MV_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_dz_BT_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ebu_KE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ee_GH_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ee_TG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_el_CY_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_el_GR_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_AS_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_AU_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_BE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_BW_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_BZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_CA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_Dsrt_US_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_GB_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_GU_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_HK_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_IE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_IN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_JM_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_MH_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_MP_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_MT_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_MU_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_NA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_NZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_PH_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_PK_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_SG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_TT_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_UM_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_US_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_US_POSIX_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_VI_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_ZA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_ZW_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_en_ZZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_es_AR_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_es_BO_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_es_CL_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_es_CO_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_es_CR_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_es_DO_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_es_EC_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_es_ES_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_es_GQ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_es_GT_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_es_HN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_es_MX_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_es_NI_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_es_PA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_es_PE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_es_PR_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_es_PY_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_es_SV_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_es_US_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_es_UY_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_es_VE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_et_EE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_eu_ES_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fa_AF_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fa_IR_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ff_SN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fi_FI_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fil_PH_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fo_FO_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_BE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_BF_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_BI_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_BJ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_BL_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_CA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_CD_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_CF_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_CG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_CH_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_CI_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_CM_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_DJ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_FR_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_GA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_GN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_GP_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_GQ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_KM_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_LU_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_MC_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_MF_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_MG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_ML_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_MQ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_NE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_RE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_RW_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_SN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_TD_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fr_TG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_fur_IT_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ga_IE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_gaa_GH_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_gez_ER_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_gez_ET_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_gl_ES_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_gsw_CH_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_gu_IN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_guz_KE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_gv_GB_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ha_Arab_NG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ha_Arab_SD_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ha_GH_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ha_Latn_GH_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ha_Latn_NE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ha_Latn_NG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ha_NE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ha_NG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ha_SD_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_haw_US_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_he_IL_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_hi_IN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_hr_HR_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_hu_HU_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_hy_AM_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_id_ID_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ig_NG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ii_CN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_is_IS_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_it_CH_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_it_IT_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ja_JP_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_jmc_TZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ka_GE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_kab_DZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_kaj_NG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_kam_KE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_kcg_NG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_kde_TZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_kea_CV_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_kfo_CI_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_khq_ML_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ki_KE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_kk_Cyrl_KZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_kk_KZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_kl_GL_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_kln_KE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_km_KH_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_kn_IN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ko_KR_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_kok_IN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_kpe_GN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_kpe_LR_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ksb_TZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ksh_DE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ku_Arab_IQ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ku_Arab_IR_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ku_IQ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ku_IR_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ku_Latn_SY_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ku_Latn_TR_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ku_SY_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ku_TR_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_kw_GB_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ky_KG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_lag_TZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_lg_UG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ln_CD_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ln_CG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_lo_LA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_lt_LT_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_luo_KE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_luy_KE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_lv_LV_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_mas_KE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_mas_TZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_mer_KE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_mfe_MU_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_mg_MG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_mi_NZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_mk_MK_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ml_IN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_mn_CN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_mn_Cyrl_MN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_mn_MN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_mn_Mong_CN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_mr_IN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ms_BN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ms_MY_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_mt_MT_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_my_MM_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_naq_NA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_nb_NO_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_nd_ZW_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_nds_DE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ne_IN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ne_NP_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_nl_BE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_nl_NL_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_nn_NO_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_nr_ZA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_nso_ZA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ny_MW_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_nyn_UG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_oc_FR_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_om_ET_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_om_KE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_or_IN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_pa_Arab_PK_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_pa_Guru_IN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_pa_IN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_pa_PK_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_pl_PL_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ps_AF_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_pt_AO_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_pt_BR_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_pt_GW_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_pt_MZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_pt_PT_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_rm_CH_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ro_MD_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ro_RO_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_rof_TZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ru_MD_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ru_RU_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ru_UA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_rw_RW_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_rwk_TZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sa_IN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_saq_KE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_se_FI_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_se_NO_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_seh_MZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ses_ML_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sg_CF_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sh_BA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sh_CS_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sh_YU_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_shi_Latn_MA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_shi_MA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_shi_Tfng_MA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_si_LK_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sid_ET_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sk_SK_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sl_SI_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sn_ZW_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_so_DJ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_so_ET_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_so_KE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_so_SO_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sq_AL_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sr_BA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sr_CS_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sr_Cyrl_BA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sr_Cyrl_CS_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sr_Cyrl_ME_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sr_Cyrl_RS_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sr_Cyrl_YU_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sr_Latn_BA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sr_Latn_CS_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sr_Latn_ME_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sr_Latn_RS_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sr_Latn_YU_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sr_ME_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sr_RS_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sr_YU_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ss_SZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ss_ZA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ssy_ER_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_st_LS_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_st_ZA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sv_FI_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sv_SE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sw_KE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_sw_TZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_syr_SY_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ta_IN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ta_LK_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_te_IN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_teo_KE_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_teo_UG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_tg_Cyrl_TJ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_tg_TJ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_th_TH_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ti_ER_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ti_ET_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_tig_ER_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_tl_PH_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_tn_ZA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_to_TO_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_tr_TR_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_trv_TW_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ts_ZA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_tt_RU_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_tzm_Latn_MA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_tzm_MA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ug_Arab_CN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ug_CN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_uk_UA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ur_IN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ur_PK_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_uz_AF_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_uz_Arab_AF_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_uz_Cyrl_UZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_uz_Latn_UZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_uz_UZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_ve_ZA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_vi_VN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_vun_TZ_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_wal_ET_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_wo_Latn_SN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_wo_SN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_xh_ZA_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_xog_UG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_yo_NG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_zh_CN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_zh_Hans_CN_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_zh_Hans_HK_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_zh_Hans_MO_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_zh_Hans_SG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_zh_Hant_HK_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_zh_Hant_MO_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_zh_Hant_TW_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_zh_HK_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_zh_MO_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_zh_SG_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_zh_TW_locale;
extern const __SEGGER_RTL_locale_data_t __SEGGER_RTL_zu_ZA_locale;

extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_ascii;
extern const __SEGGER_RTL_locale_codeset_t __SEGGER_RTL_codeset_utf8;

extern const char __SEGGER_RTL_data_utf8_period [];
extern const char __SEGGER_RTL_data_utf8_comma [];
extern const char __SEGGER_RTL_data_utf8_space [];
extern const char __SEGGER_RTL_data_utf8_plus [];
extern const char __SEGGER_RTL_data_utf8_minus [];
extern const char __SEGGER_RTL_data_empty_string[];
# 548 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL.h" 3 4
int __SEGGER_RTL_X_set_time_of_day (const struct timeval *__tp);
int __SEGGER_RTL_X_get_time_of_day (struct timeval *__tp);
const __SEGGER_RTL_locale_t * __SEGGER_RTL_X_find_locale (const char *__locale);
__SEGGER_RTL_FILE * __SEGGER_RTL_X_file_open (const char *__filename, const char *__mode);
int __SEGGER_RTL_X_file_stat (__SEGGER_RTL_FILE *__stream);
int __SEGGER_RTL_X_file_read (__SEGGER_RTL_FILE *__stream, char *__s, unsigned __len);
int __SEGGER_RTL_X_file_write (__SEGGER_RTL_FILE *__stream, const char *__s, unsigned __len);
int __SEGGER_RTL_X_file_unget (__SEGGER_RTL_FILE *__stream, int __c);
int __SEGGER_RTL_X_file_close (__SEGGER_RTL_FILE *__stream);
int __SEGGER_RTL_X_file_error (__SEGGER_RTL_FILE *__stream);
int __SEGGER_RTL_X_file_flush (__SEGGER_RTL_FILE *__stream);
int __SEGGER_RTL_X_file_eof (__SEGGER_RTL_FILE *__stream);
void __SEGGER_RTL_X_file_clrerr (__SEGGER_RTL_FILE *__stream);
int __SEGGER_RTL_X_file_seek (__SEGGER_RTL_FILE *__stream, long __offset, int __whence);
int __SEGGER_RTL_X_file_getpos (__SEGGER_RTL_FILE *__stream, long *__pos);
int __SEGGER_RTL_X_file_bufsize (__SEGGER_RTL_FILE *__stream);
__SEGGER_RTL_FILE * __SEGGER_RTL_X_file_tmpfile (void);
char * __SEGGER_RTL_X_file_tmpnam (char *__s, unsigned __max);
int __SEGGER_RTL_X_file_remove (const char *__filename);
int __SEGGER_RTL_X_file_rename (const char *__old, const char *__new);
void __attribute__((__weak__)) __SEGGER_RTL_X_heap_lock (void);
void __attribute__((__weak__)) __SEGGER_RTL_X_heap_unlock (void);
int __attribute__((__weak__)) __SEGGER_RTL_X_atomic_lock (void);
void __attribute__((__weak__)) __SEGGER_RTL_X_atomic_unlock (int __state);
int __attribute__((__weak__)) __SEGGER_RTL_X_atomic_is_lock_free (unsigned int __size, void *__ptr);
void __attribute__((__weak__)) __SEGGER_RTL_X_atomic_synchronize (void);
volatile int * __aeabi_errno_addr (void);
void __aeabi_assert (const char *__expression, const char *__filename, int __line);
# 584 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/__SEGGER_RTL.h" 3 4
void __SEGGER_RTL_execute_at_exit_fns (void);
void __SEGGER_RTL_set_locale_name_buffer (char *__buffer);
int __SEGGER_RTL_mb_max (const struct __SEGGER_RTL_POSIX_locale_s *__loc);
int __SEGGER_RTL_mb_cur_max (void);
void __SEGGER_RTL_init_heap (void *__ptr, unsigned int __size);
# 21 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/stdint.h" 2 3 4
# 223 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/stdint.h" 3 4
typedef signed char int8_t;
typedef unsigned char uint8_t;

typedef signed short int16_t;
typedef unsigned short uint16_t;
typedef signed int int32_t;
typedef unsigned int uint32_t;

typedef signed long long int64_t;
typedef unsigned long long uint64_t;

typedef int8_t int_least8_t;
typedef int16_t int_least16_t;
typedef int32_t int_least32_t;
typedef int64_t int_least64_t;

typedef uint8_t uint_least8_t;
typedef uint16_t uint_least16_t;
typedef uint32_t uint_least32_t;
typedef uint64_t uint_least64_t;

typedef int32_t int_fast8_t;
typedef int32_t int_fast16_t;
typedef int32_t int_fast32_t;
typedef int64_t int_fast64_t;

typedef uint32_t uint_fast8_t;
typedef uint32_t uint_fast16_t;
typedef uint32_t uint_fast32_t;
typedef uint64_t uint_fast64_t;


typedef int32_t intptr_t;
typedef uint32_t uintptr_t;





typedef int64_t intmax_t;
typedef uint64_t uintmax_t;
# 5 "C:\\Users\\haris\\Downloads\\DeviceDownload\\nRF5_SDK_17.1.0_ddde560\\nRF5_SDK_17.1.0_ddde560\\examples\\My Project\\adafruit_tag_reader\\nfc_spk_eeprom\\nfc_spk_eeprom.h" 2
# 1 "../../../../../../components/libraries/util/sdk_errors.h" 1
# 73 "../../../../../../components/libraries/util/sdk_errors.h"
# 1 "../../../../../../components/drivers_nrf/nrf_soc_nosd/nrf_error.h" 1
# 74 "../../../../../../components/libraries/util/sdk_errors.h" 2
# 159 "../../../../../../components/libraries/util/sdk_errors.h"

# 159 "../../../../../../components/libraries/util/sdk_errors.h"
typedef uint32_t ret_code_t;
# 6 "C:\\Users\\haris\\Downloads\\DeviceDownload\\nRF5_SDK_17.1.0_ddde560\\nRF5_SDK_17.1.0_ddde560\\examples\\My Project\\adafruit_tag_reader\\nfc_spk_eeprom\\nfc_spk_eeprom.h" 2
# 1 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 1
# 44 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
# 1 "../../../../../../modules/nrfx/nrfx.h" 1
# 44 "../../../../../../modules/nrfx/nrfx.h"
# 1 "../../../../../../integration/nrfx/nrfx_config.h" 1
# 45 "../../../../../../integration/nrfx/nrfx_config.h"
# 1 "../config/sdk_config.h" 1
# 46 "../../../../../../integration/nrfx/nrfx_config.h" 2
# 45 "../../../../../../modules/nrfx/nrfx.h" 2
# 1 "../../../../../../modules/nrfx/drivers/nrfx_common.h" 1
# 45 "../../../../../../modules/nrfx/drivers/nrfx_common.h"
# 1 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/stddef.h" 1 3 4
# 72 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/stddef.h" 3 4

# 72 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/stddef.h" 3 4
typedef unsigned int size_t;
# 86 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/stddef.h" 3 4
typedef unsigned int wchar_t;
# 97 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/stddef.h" 3 4
typedef int ptrdiff_t;
# 46 "../../../../../../modules/nrfx/drivers/nrfx_common.h" 2
# 1 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/stdbool.h" 1 3 4
# 47 "../../../../../../modules/nrfx/drivers/nrfx_common.h" 2

# 1 "../../../../../../modules/nrfx/mdk/nrf.h" 1
# 160 "../../../../../../modules/nrfx/mdk/nrf.h"
# 1 "../../../../../../modules/nrfx/mdk/nrf52.h" 1
# 78 "../../../../../../modules/nrfx/mdk/nrf52.h"

# 78 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef enum {

  Reset_IRQn = -15,
  NonMaskableInt_IRQn = -14,
  HardFault_IRQn = -13,
  MemoryManagement_IRQn = -12,

  BusFault_IRQn = -11,

  UsageFault_IRQn = -10,
  SVCall_IRQn = -5,
  DebugMonitor_IRQn = -4,
  PendSV_IRQn = -2,
  SysTick_IRQn = -1,

  POWER_CLOCK_IRQn = 0,
  RADIO_IRQn = 1,
  UARTE0_UART0_IRQn = 2,
  SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn= 3,
  SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn= 4,
  NFCT_IRQn = 5,
  GPIOTE_IRQn = 6,
  SAADC_IRQn = 7,
  TIMER0_IRQn = 8,
  TIMER1_IRQn = 9,
  TIMER2_IRQn = 10,
  RTC0_IRQn = 11,
  TEMP_IRQn = 12,
  RNG_IRQn = 13,
  ECB_IRQn = 14,
  CCM_AAR_IRQn = 15,
  WDT_IRQn = 16,
  RTC1_IRQn = 17,
  QDEC_IRQn = 18,
  COMP_LPCOMP_IRQn = 19,
  SWI0_EGU0_IRQn = 20,
  SWI1_EGU1_IRQn = 21,
  SWI2_EGU2_IRQn = 22,
  SWI3_EGU3_IRQn = 23,
  SWI4_EGU4_IRQn = 24,
  SWI5_EGU5_IRQn = 25,
  TIMER3_IRQn = 26,
  TIMER4_IRQn = 27,
  PWM0_IRQn = 28,
  PDM_IRQn = 29,
  MWU_IRQn = 32,
  PWM1_IRQn = 33,
  PWM2_IRQn = 34,
  SPIM2_SPIS2_SPI2_IRQn = 35,
  RTC2_IRQn = 36,
  I2S_IRQn = 37,
  FPU_IRQn = 38
} IRQn_Type;
# 150 "../../../../../../modules/nrfx/mdk/nrf52.h"
# 1 "../../../../../../components/toolchain/cmsis/include/core_cm4.h" 1
# 63 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
# 1 "../../../../../../components/toolchain/cmsis/include/cmsis_version.h" 1
# 64 "../../../../../../components/toolchain/cmsis/include/core_cm4.h" 2
# 162 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
# 1 "../../../../../../components/toolchain/cmsis/include/cmsis_compiler.h" 1
# 54 "../../../../../../components/toolchain/cmsis/include/cmsis_compiler.h"
# 1 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h" 1
# 29 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wunused-parameter"
# 71 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
  struct __attribute__((packed)) T_UINT32 { uint32_t v; };
#pragma GCC diagnostic pop



#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
  struct __attribute__((packed, aligned(1))) T_UINT16_WRITE { uint16_t v; };
#pragma GCC diagnostic pop



#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
  struct __attribute__((packed, aligned(1))) T_UINT16_READ { uint16_t v; };
#pragma GCC diagnostic pop



#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
  struct __attribute__((packed, aligned(1))) T_UINT32_WRITE { uint32_t v; };
#pragma GCC diagnostic pop



#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
  struct __attribute__((packed, aligned(1))) T_UINT32_READ { uint32_t v; };
#pragma GCC diagnostic pop
# 131 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline __attribute__((__noreturn__)) void __cmsis_start(void)
{
  extern void _start(void) __attribute__((__noreturn__));

  typedef struct {
    uint32_t const* src;
    uint32_t* dest;
    uint32_t wlen;
  } __copy_table_t;

  typedef struct {
    uint32_t* dest;
    uint32_t wlen;
  } __zero_table_t;

  extern const __copy_table_t __copy_table_start__;
  extern const __copy_table_t __copy_table_end__;
  extern const __zero_table_t __zero_table_start__;
  extern const __zero_table_t __zero_table_end__;

  for (__copy_table_t const* pTable = &__copy_table_start__; pTable < &__copy_table_end__; ++pTable) {
    for(uint32_t i=0u; i<pTable->wlen; ++i) {
      pTable->dest[i] = pTable->src[i];
    }
  }

  for (__zero_table_t const* pTable = &__zero_table_start__; pTable < &__zero_table_end__; ++pTable) {
    for(uint32_t i=0u; i<pTable->wlen; ++i) {
      pTable->dest[i] = 0u;
    }
  }

  _start();
}
# 196 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __enable_irq(void)
{
  __asm volatile ("cpsie i" : : : "memory");
}







__attribute__((always_inline)) static inline void __disable_irq(void)
{
  __asm volatile ("cpsid i" : : : "memory");
}







__attribute__((always_inline)) static inline uint32_t __get_CONTROL(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, control" : "=r" (result) );
  return(result);
}
# 248 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __set_CONTROL(uint32_t control)
{
  __asm volatile ("MSR control, %0" : : "r" (control) : "memory");
}
# 272 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __get_IPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, ipsr" : "=r" (result) );
  return(result);
}







__attribute__((always_inline)) static inline uint32_t __get_APSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, apsr" : "=r" (result) );
  return(result);
}







__attribute__((always_inline)) static inline uint32_t __get_xPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, xpsr" : "=r" (result) );
  return(result);
}







__attribute__((always_inline)) static inline uint32_t __get_PSP(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, psp" : "=r" (result) );
  return(result);
}
# 344 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __set_PSP(uint32_t topOfProcStack)
{
  __asm volatile ("MSR psp, %0" : : "r" (topOfProcStack) : );
}
# 368 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __get_MSP(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, msp" : "=r" (result) );
  return(result);
}
# 398 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __set_MSP(uint32_t topOfMainStack)
{
  __asm volatile ("MSR msp, %0" : : "r" (topOfMainStack) : );
}
# 449 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __get_PRIMASK(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, primask" : "=r" (result) :: "memory");
  return(result);
}
# 479 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __set_PRIMASK(uint32_t priMask)
{
  __asm volatile ("MSR primask, %0" : : "r" (priMask) : "memory");
}
# 506 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __enable_fault_irq(void)
{
  __asm volatile ("cpsie f" : : : "memory");
}







__attribute__((always_inline)) static inline void __disable_fault_irq(void)
{
  __asm volatile ("cpsid f" : : : "memory");
}







__attribute__((always_inline)) static inline uint32_t __get_BASEPRI(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, basepri" : "=r" (result) );
  return(result);
}
# 558 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __set_BASEPRI(uint32_t basePri)
{
  __asm volatile ("MSR basepri, %0" : : "r" (basePri) : "memory");
}
# 583 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __set_BASEPRI_MAX(uint32_t basePri)
{
  __asm volatile ("MSR basepri_max, %0" : : "r" (basePri) : "memory");
}







__attribute__((always_inline)) static inline uint32_t __get_FAULTMASK(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, faultmask" : "=r" (result) );
  return(result);
}
# 624 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __set_FAULTMASK(uint32_t faultMask)
{
  __asm volatile ("MSR faultmask, %0" : : "r" (faultMask) : "memory");
}
# 833 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __get_FPSCR(void)
{






  return __builtin_arm_get_fpscr();
# 851 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
}







__attribute__((always_inline)) static inline void __set_FPSCR(uint32_t fpscr)
{






  __builtin_arm_set_fpscr(fpscr);






}
# 933 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __ISB(void)
{
  __asm volatile ("isb 0xF":::"memory");
}







__attribute__((always_inline)) static inline void __DSB(void)
{
  __asm volatile ("dsb 0xF":::"memory");
}







__attribute__((always_inline)) static inline void __DMB(void)
{
  __asm volatile ("dmb 0xF":::"memory");
}
# 967 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __REV(uint32_t value)
{

  return __builtin_bswap32(value);






}
# 986 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __REV16(uint32_t value)
{
  uint32_t result;

  __asm volatile ("rev16 %0, %1" : "=r" (result) : "r" (value) );
  return result;
}
# 1001 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline int16_t __REVSH(int16_t value)
{

  return (int16_t)__builtin_bswap16(value);






}
# 1021 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __ROR(uint32_t op1, uint32_t op2)
{
  op2 %= 32U;
  if (op2 == 0U)
  {
    return op1;
  }
  return (op1 >> op2) | (op1 << (32U - op2));
}
# 1048 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __RBIT(uint32_t value)
{
  uint32_t result;




   __asm volatile ("rbit %0, %1" : "=r" (result) : "r" (value) );
# 1068 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
  return result;
}
# 1078 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint8_t __CLZ(uint32_t value)
{
# 1089 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
  if (value == 0U)
  {
    return 32U;
  }
  return __builtin_clz(value);
}
# 1107 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint8_t __LDREXB(volatile uint8_t *addr)
{
    uint32_t result;


   __asm volatile ("ldrexb %0, %1" : "=r" (result) : "Q" (*addr) );






   return ((uint8_t) result);
}
# 1129 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint16_t __LDREXH(volatile uint16_t *addr)
{
    uint32_t result;


   __asm volatile ("ldrexh %0, %1" : "=r" (result) : "Q" (*addr) );






   return ((uint16_t) result);
}
# 1151 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __LDREXW(volatile uint32_t *addr)
{
    uint32_t result;

   __asm volatile ("ldrex %0, %1" : "=r" (result) : "Q" (*addr) );
   return(result);
}
# 1168 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __STREXB(uint8_t value, volatile uint8_t *addr)
{
   uint32_t result;

   __asm volatile ("strexb %0, %2, %1" : "=&r" (result), "=Q" (*addr) : "r" ((uint32_t)value) );
   return(result);
}
# 1185 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __STREXH(uint16_t value, volatile uint16_t *addr)
{
   uint32_t result;

   __asm volatile ("strexh %0, %2, %1" : "=&r" (result), "=Q" (*addr) : "r" ((uint32_t)value) );
   return(result);
}
# 1202 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __STREXW(uint32_t value, volatile uint32_t *addr)
{
   uint32_t result;

   __asm volatile ("strex %0, %2, %1" : "=&r" (result), "=Q" (*addr) : "r" (value) );
   return(result);
}






__attribute__((always_inline)) static inline void __CLREX(void)
{
  __asm volatile ("clrex" ::: "memory");
}
# 1268 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __RRX(uint32_t value)
{
  uint32_t result;

  __asm volatile ("rrx %0, %1" : "=r" (result) : "r" (value) );
  return(result);
}
# 1283 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint8_t __LDRBT(volatile uint8_t *ptr)
{
    uint32_t result;


   __asm volatile ("ldrbt %0, %1" : "=r" (result) : "Q" (*ptr) );






   return ((uint8_t) result);
}
# 1305 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint16_t __LDRHT(volatile uint16_t *ptr)
{
    uint32_t result;


   __asm volatile ("ldrht %0, %1" : "=r" (result) : "Q" (*ptr) );






   return ((uint16_t) result);
}
# 1327 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __LDRT(volatile uint32_t *ptr)
{
    uint32_t result;

   __asm volatile ("ldrt %0, %1" : "=r" (result) : "Q" (*ptr) );
   return(result);
}
# 1342 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __STRBT(uint8_t value, volatile uint8_t *ptr)
{
   __asm volatile ("strbt %1, %0" : "=Q" (*ptr) : "r" ((uint32_t)value) );
}
# 1354 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __STRHT(uint16_t value, volatile uint16_t *ptr)
{
   __asm volatile ("strht %1, %0" : "=Q" (*ptr) : "r" ((uint32_t)value) );
}
# 1366 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __STRT(uint32_t value, volatile uint32_t *ptr)
{
   __asm volatile ("strt %1, %0" : "=Q" (*ptr) : "r" (value) );
}
# 1621 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __SADD8(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("sadd8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __QADD8(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("qadd8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __SHADD8(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("shadd8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __UADD8(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("uadd8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __UQADD8(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("uqadd8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __UHADD8(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("uhadd8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}


__attribute__((always_inline)) static inline uint32_t __SSUB8(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("ssub8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __QSUB8(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("qsub8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __SHSUB8(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("shsub8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __USUB8(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("usub8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __UQSUB8(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("uqsub8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __UHSUB8(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("uhsub8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}


__attribute__((always_inline)) static inline uint32_t __SADD16(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("sadd16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __QADD16(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("qadd16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __SHADD16(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("shadd16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __UADD16(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("uadd16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __UQADD16(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("uqadd16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __UHADD16(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("uhadd16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __SSUB16(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("ssub16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __QSUB16(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("qsub16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __SHSUB16(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("shsub16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __USUB16(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("usub16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __UQSUB16(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("uqsub16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __UHSUB16(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("uhsub16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __SASX(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("sasx %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __QASX(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("qasx %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __SHASX(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("shasx %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __UASX(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("uasx %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __UQASX(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("uqasx %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __UHASX(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("uhasx %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __SSAX(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("ssax %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __QSAX(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("qsax %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __SHSAX(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("shsax %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __USAX(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("usax %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __UQSAX(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("uqsax %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __UHSAX(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("uhsax %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __USAD8(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("usad8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __USADA8(uint32_t op1, uint32_t op2, uint32_t op3)
{
  uint32_t result;

  __asm volatile ("usada8 %0, %1, %2, %3" : "=r" (result) : "r" (op1), "r" (op2), "r" (op3) );
  return(result);
}
# 1941 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __UXTB16(uint32_t op1)
{
  uint32_t result;

  __asm volatile ("uxtb16 %0, %1" : "=r" (result) : "r" (op1));
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __UXTAB16(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("uxtab16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __SXTB16(uint32_t op1)
{
  uint32_t result;

  __asm volatile ("sxtb16 %0, %1" : "=r" (result) : "r" (op1));
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __SXTAB16(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("sxtab16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __SMUAD (uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("smuad %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __SMUADX (uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("smuadx %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __SMLAD (uint32_t op1, uint32_t op2, uint32_t op3)
{
  uint32_t result;

  __asm volatile ("smlad %0, %1, %2, %3" : "=r" (result) : "r" (op1), "r" (op2), "r" (op3) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __SMLADX (uint32_t op1, uint32_t op2, uint32_t op3)
{
  uint32_t result;

  __asm volatile ("smladx %0, %1, %2, %3" : "=r" (result) : "r" (op1), "r" (op2), "r" (op3) );
  return(result);
}

__attribute__((always_inline)) static inline uint64_t __SMLALD (uint32_t op1, uint32_t op2, uint64_t acc)
{
  union llreg_u{
    uint32_t w32[2];
    uint64_t w64;
  } llr;
  llr.w64 = acc;


  __asm volatile ("smlald %0, %1, %2, %3" : "=r" (llr.w32[0]), "=r" (llr.w32[1]): "r" (op1), "r" (op2) , "0" (llr.w32[0]), "1" (llr.w32[1]) );




  return(llr.w64);
}

__attribute__((always_inline)) static inline uint64_t __SMLALDX (uint32_t op1, uint32_t op2, uint64_t acc)
{
  union llreg_u{
    uint32_t w32[2];
    uint64_t w64;
  } llr;
  llr.w64 = acc;


  __asm volatile ("smlaldx %0, %1, %2, %3" : "=r" (llr.w32[0]), "=r" (llr.w32[1]): "r" (op1), "r" (op2) , "0" (llr.w32[0]), "1" (llr.w32[1]) );




  return(llr.w64);
}

__attribute__((always_inline)) static inline uint32_t __SMUSD (uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("smusd %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __SMUSDX (uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("smusdx %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __SMLSD (uint32_t op1, uint32_t op2, uint32_t op3)
{
  uint32_t result;

  __asm volatile ("smlsd %0, %1, %2, %3" : "=r" (result) : "r" (op1), "r" (op2), "r" (op3) );
  return(result);
}

__attribute__((always_inline)) static inline uint32_t __SMLSDX (uint32_t op1, uint32_t op2, uint32_t op3)
{
  uint32_t result;

  __asm volatile ("smlsdx %0, %1, %2, %3" : "=r" (result) : "r" (op1), "r" (op2), "r" (op3) );
  return(result);
}

__attribute__((always_inline)) static inline uint64_t __SMLSLD (uint32_t op1, uint32_t op2, uint64_t acc)
{
  union llreg_u{
    uint32_t w32[2];
    uint64_t w64;
  } llr;
  llr.w64 = acc;


  __asm volatile ("smlsld %0, %1, %2, %3" : "=r" (llr.w32[0]), "=r" (llr.w32[1]): "r" (op1), "r" (op2) , "0" (llr.w32[0]), "1" (llr.w32[1]) );




  return(llr.w64);
}

__attribute__((always_inline)) static inline uint64_t __SMLSLDX (uint32_t op1, uint32_t op2, uint64_t acc)
{
  union llreg_u{
    uint32_t w32[2];
    uint64_t w64;
  } llr;
  llr.w64 = acc;


  __asm volatile ("smlsldx %0, %1, %2, %3" : "=r" (llr.w32[0]), "=r" (llr.w32[1]): "r" (op1), "r" (op2) , "0" (llr.w32[0]), "1" (llr.w32[1]) );




  return(llr.w64);
}

__attribute__((always_inline)) static inline uint32_t __SEL (uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __asm volatile ("sel %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline int32_t __QADD( int32_t op1, int32_t op2)
{
  int32_t result;

  __asm volatile ("qadd %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__((always_inline)) static inline int32_t __QSUB( int32_t op1, int32_t op2)
{
  int32_t result;

  __asm volatile ("qsub %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}
# 2154 "../../../../../../components/toolchain/cmsis/include/cmsis_gcc.h"
__attribute__((always_inline)) static inline int32_t __SMMLA (int32_t op1, int32_t op2, int32_t op3)
{
 int32_t result;

 __asm volatile ("smmla %0, %1, %2, %3" : "=r" (result): "r" (op1), "r" (op2), "r" (op3) );
 return(result);
}





#pragma GCC diagnostic pop
# 55 "../../../../../../components/toolchain/cmsis/include/cmsis_compiler.h" 2
# 163 "../../../../../../components/toolchain/cmsis/include/core_cm4.h" 2
# 259 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
typedef union
{
  struct
  {
    uint32_t _reserved0:16;
    uint32_t GE:4;
    uint32_t _reserved1:7;
    uint32_t Q:1;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} APSR_Type;
# 298 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
typedef union
{
  struct
  {
    uint32_t ISR:9;
    uint32_t _reserved0:23;
  } b;
  uint32_t w;
} IPSR_Type;
# 316 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
typedef union
{
  struct
  {
    uint32_t ISR:9;
    uint32_t _reserved0:1;
    uint32_t ICI_IT_1:6;
    uint32_t GE:4;
    uint32_t _reserved1:4;
    uint32_t T:1;
    uint32_t ICI_IT_2:2;
    uint32_t Q:1;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} xPSR_Type;
# 371 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
typedef union
{
  struct
  {
    uint32_t nPRIV:1;
    uint32_t SPSEL:1;
    uint32_t FPCA:1;
    uint32_t _reserved0:29;
  } b;
  uint32_t w;
} CONTROL_Type;
# 406 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
typedef struct
{
  volatile uint32_t ISER[8U];
        uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];
        uint32_t RESERVED1[24U];
  volatile uint32_t ISPR[8U];
        uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];
        uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];
        uint32_t RESERVED4[56U];
  volatile uint8_t IP[240U];
        uint32_t RESERVED5[644U];
  volatile uint32_t STIR;
} NVIC_Type;
# 440 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
typedef struct
{
  volatile const uint32_t CPUID;
  volatile uint32_t ICSR;
  volatile uint32_t VTOR;
  volatile uint32_t AIRCR;
  volatile uint32_t SCR;
  volatile uint32_t CCR;
  volatile uint8_t SHP[12U];
  volatile uint32_t SHCSR;
  volatile uint32_t CFSR;
  volatile uint32_t HFSR;
  volatile uint32_t DFSR;
  volatile uint32_t MMFAR;
  volatile uint32_t BFAR;
  volatile uint32_t AFSR;
  volatile const uint32_t PFR[2U];
  volatile const uint32_t DFR;
  volatile const uint32_t ADR;
  volatile const uint32_t MMFR[4U];
  volatile const uint32_t ISAR[5U];
        uint32_t RESERVED0[5U];
  volatile uint32_t CPACR;
} SCB_Type;
# 719 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile const uint32_t ICTR;
  volatile uint32_t ACTLR;
} SCnSCB_Type;
# 759 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t LOAD;
  volatile uint32_t VAL;
  volatile const uint32_t CALIB;
} SysTick_Type;
# 811 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
typedef struct
{
  volatile union
  {
    volatile uint8_t u8;
    volatile uint16_t u16;
    volatile uint32_t u32;
  } PORT [32U];
        uint32_t RESERVED0[864U];
  volatile uint32_t TER;
        uint32_t RESERVED1[15U];
  volatile uint32_t TPR;
        uint32_t RESERVED2[15U];
  volatile uint32_t TCR;
        uint32_t RESERVED3[32U];
        uint32_t RESERVED4[43U];
  volatile uint32_t LAR;
  volatile const uint32_t LSR;
        uint32_t RESERVED5[6U];
  volatile const uint32_t PID4;
  volatile const uint32_t PID5;
  volatile const uint32_t PID6;
  volatile const uint32_t PID7;
  volatile const uint32_t PID0;
  volatile const uint32_t PID1;
  volatile const uint32_t PID2;
  volatile const uint32_t PID3;
  volatile const uint32_t CID0;
  volatile const uint32_t CID1;
  volatile const uint32_t CID2;
  volatile const uint32_t CID3;
} ITM_Type;
# 899 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t CYCCNT;
  volatile uint32_t CPICNT;
  volatile uint32_t EXCCNT;
  volatile uint32_t SLEEPCNT;
  volatile uint32_t LSUCNT;
  volatile uint32_t FOLDCNT;
  volatile const uint32_t PCSR;
  volatile uint32_t COMP0;
  volatile uint32_t MASK0;
  volatile uint32_t FUNCTION0;
        uint32_t RESERVED0[1U];
  volatile uint32_t COMP1;
  volatile uint32_t MASK1;
  volatile uint32_t FUNCTION1;
        uint32_t RESERVED1[1U];
  volatile uint32_t COMP2;
  volatile uint32_t MASK2;
  volatile uint32_t FUNCTION2;
        uint32_t RESERVED2[1U];
  volatile uint32_t COMP3;
  volatile uint32_t MASK3;
  volatile uint32_t FUNCTION3;
} DWT_Type;
# 1046 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
typedef struct
{
  volatile const uint32_t SSPSR;
  volatile uint32_t CSPSR;
        uint32_t RESERVED0[2U];
  volatile uint32_t ACPR;
        uint32_t RESERVED1[55U];
  volatile uint32_t SPPR;
        uint32_t RESERVED2[131U];
  volatile const uint32_t FFSR;
  volatile uint32_t FFCR;
  volatile const uint32_t FSCR;
        uint32_t RESERVED3[759U];
  volatile const uint32_t TRIGGER;
  volatile const uint32_t FIFO0;
  volatile const uint32_t ITATBCTR2;
        uint32_t RESERVED4[1U];
  volatile const uint32_t ITATBCTR0;
  volatile const uint32_t FIFO1;
  volatile uint32_t ITCTRL;
        uint32_t RESERVED5[39U];
  volatile uint32_t CLAIMSET;
  volatile uint32_t CLAIMCLR;
        uint32_t RESERVED7[8U];
  volatile const uint32_t DEVID;
  volatile const uint32_t DEVTYPE;
} TPI_Type;
# 1208 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
typedef struct
{
  volatile const uint32_t TYPE;
  volatile uint32_t CTRL;
  volatile uint32_t RNR;
  volatile uint32_t RBAR;
  volatile uint32_t RASR;
  volatile uint32_t RBAR_A1;
  volatile uint32_t RASR_A1;
  volatile uint32_t RBAR_A2;
  volatile uint32_t RASR_A2;
  volatile uint32_t RBAR_A3;
  volatile uint32_t RASR_A3;
} MPU_Type;
# 1304 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile uint32_t FPCCR;
  volatile uint32_t FPCAR;
  volatile uint32_t FPDSCR;
  volatile const uint32_t MVFR0;
  volatile const uint32_t MVFR1;
  volatile const uint32_t MVFR2;
} FPU_Type;
# 1416 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
typedef struct
{
  volatile uint32_t DHCSR;
  volatile uint32_t DCRSR;
  volatile uint32_t DCRDR;
  volatile uint32_t DEMCR;
} CoreDebug_Type;
# 1648 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
static inline void __NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);

  reg_value = ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;
  reg_value &= ~((uint32_t)((0xFFFFUL << 16U) | (7UL << 8U)));
  reg_value = (reg_value |
                ((uint32_t)0x5FAUL << 16U) |
                (PriorityGroupTmp << 8U) );
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR = reg_value;
}







static inline uint32_t __NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) >> 8U));
}
# 1679 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
static inline void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    __asm volatile("":::"memory");
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __asm volatile("":::"memory");
  }
}
# 1698 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
static inline uint32_t __NVIC_GetEnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}
# 1717 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
static inline void __NVIC_DisableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __DSB();
    __ISB();
  }
}
# 1736 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
static inline uint32_t __NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}
# 1755 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
static inline void __NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}
# 1770 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
static inline void __NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}
# 1787 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
static inline uint32_t __NVIC_GetActive(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}
# 1809 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
static inline void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)IRQn)] = (uint8_t)((priority << (8U - 3)) & (uint32_t)0xFFUL);
  }
  else
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - 3)) & (uint32_t)0xFFUL);
  }
}
# 1831 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
static inline uint32_t __NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) >= 0)
  {
    return(((uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)IRQn)] >> (8U - 3)));
  }
  else
  {
    return(((uint32_t)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)IRQn) & 0xFUL)-4UL] >> (8U - 3)));
  }
}
# 1856 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
static inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(3)) ? (uint32_t)(3) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits = ((PriorityGroupTmp + (uint32_t)(3)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(3));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority & (uint32_t)((1UL << (SubPriorityBits )) - 1UL)))
         );
}
# 1883 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
static inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(3)) ? (uint32_t)(3) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits = ((PriorityGroupTmp + (uint32_t)(3)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(3));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority = (Priority ) & (uint32_t)((1UL << (SubPriorityBits )) - 1UL);
}
# 1906 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
static inline void __NVIC_SetVector(IRQn_Type IRQn, uint32_t vector)
{
  uint32_t vectors = (uint32_t )((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR;
  (* (int *) (vectors + ((int32_t)IRQn + 16) * 4)) = vector;

}
# 1922 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
static inline uint32_t __NVIC_GetVector(IRQn_Type IRQn)
{
  uint32_t vectors = (uint32_t )((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR;
  return (uint32_t)(* (int *) (vectors + ((int32_t)IRQn + 16) * 4));
}






__attribute__((__noreturn__)) static inline void __NVIC_SystemReset(void)
{
  __DSB();

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR = (uint32_t)((0x5FAUL << 16U) |
                           (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) |
                            (1UL << 2U) );
  __DSB();

  for(;;)
  {
    __asm volatile ("nop");
  }
}
# 1955 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
# 1 "../../../../../../components/toolchain/cmsis/include/mpu_armv7.h" 1
# 183 "../../../../../../components/toolchain/cmsis/include/mpu_armv7.h"
typedef struct {
  uint32_t RBAR;
  uint32_t RASR;
} ARM_MPU_Region_t;




static inline void ARM_MPU_Enable(uint32_t MPU_Control)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL = MPU_Control | (1UL );

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHCSR |= (1UL << 16U);

  __DSB();
  __ISB();
}



static inline void ARM_MPU_Disable(void)
{
  __DMB();

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHCSR &= ~(1UL << 16U);

  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL &= ~(1UL );
}




static inline void ARM_MPU_ClrRegion(uint32_t rnr)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RNR = rnr;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR = 0U;
}





static inline void ARM_MPU_SetRegion(uint32_t rbar, uint32_t rasr)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR = rbar;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR = rasr;
}






static inline void ARM_MPU_SetRegionEx(uint32_t rnr, uint32_t rbar, uint32_t rasr)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RNR = rnr;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR = rbar;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR = rasr;
}






static inline void ARM_MPU_OrderedMemcpy(volatile uint32_t* dst, const uint32_t* __restrict src, uint32_t len)
{
  uint32_t i;
  for (i = 0U; i < len; ++i)
  {
    dst[i] = src[i];
  }
}





static inline void ARM_MPU_Load(ARM_MPU_Region_t const* table, uint32_t cnt)
{
  const uint32_t rowWordSize = sizeof(ARM_MPU_Region_t)/4U;
  while (cnt > 4U) {
    ARM_MPU_OrderedMemcpy(&(((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR), &(table->RBAR), 4U*rowWordSize);
    table += 4U;
    cnt -= 4U;
  }
  ARM_MPU_OrderedMemcpy(&(((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR), &(table->RBAR), cnt*rowWordSize);
}
# 1956 "../../../../../../components/toolchain/cmsis/include/core_cm4.h" 2
# 1976 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
static inline uint32_t SCB_GetFPUType(void)
{
  uint32_t mvfr0;

  mvfr0 = ((FPU_Type *) ((0xE000E000UL) + 0x0F30UL) )->MVFR0;
  if ((mvfr0 & ((0xFUL << 4U) | (0xFUL << 8U))) == 0x020U)
  {
    return 1U;
  }
  else
  {
    return 0U;
  }
}
# 2017 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
static inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = (uint32_t)(ticks - 1UL);
  __NVIC_SetPriority (SysTick_IRQn, (1UL << 3) - 1UL);
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL = 0UL;
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2U) |
                   (1UL << 1U) |
                   (1UL );
  return (0UL);
}
# 2047 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
extern volatile int32_t ITM_RxBuffer;
# 2059 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
static inline uint32_t ITM_SendChar (uint32_t ch)
{
  if (((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL )) != 0UL) &&
      ((((ITM_Type *) (0xE0000000UL) )->TER & 1UL ) != 0UL) )
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0U].u32 == 0UL)
    {
      __asm volatile ("nop");
    }
    ((ITM_Type *) (0xE0000000UL) )->PORT[0U].u8 = (uint8_t)ch;
  }
  return (ch);
}
# 2080 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
static inline int32_t ITM_ReceiveChar (void)
{
  int32_t ch = -1;

  if (ITM_RxBuffer != ((int32_t)0x5AA55AA5U))
  {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = ((int32_t)0x5AA55AA5U);
  }

  return (ch);
}
# 2100 "../../../../../../components/toolchain/cmsis/include/core_cm4.h"
static inline int32_t ITM_CheckChar (void)
{

  if (ITM_RxBuffer == ((int32_t)0x5AA55AA5U))
  {
    return (0);
  }
  else
  {
    return (1);
  }
}
# 151 "../../../../../../modules/nrfx/mdk/nrf52.h" 2
# 1 "../../../../../../modules/nrfx/mdk/system_nrf52.h" 1
# 33 "../../../../../../modules/nrfx/mdk/system_nrf52.h"
extern uint32_t SystemCoreClock;
# 44 "../../../../../../modules/nrfx/mdk/system_nrf52.h"
extern void SystemInit (void);
# 55 "../../../../../../modules/nrfx/mdk/system_nrf52.h"
extern void SystemCoreClockUpdate (void);
# 152 "../../../../../../modules/nrfx/mdk/nrf52.h" 2
# 202 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile const uint32_t PART;
  volatile const uint32_t VARIANT;

  volatile const uint32_t PACKAGE;
  volatile const uint32_t RAM;
  volatile const uint32_t FLASH;
  volatile uint32_t UNUSED0[3];
} FICR_INFO_Type;





typedef struct {
  volatile const uint32_t A0;
  volatile const uint32_t A1;
  volatile const uint32_t A2;
  volatile const uint32_t A3;
  volatile const uint32_t A4;
  volatile const uint32_t A5;
  volatile const uint32_t B0;
  volatile const uint32_t B1;
  volatile const uint32_t B2;
  volatile const uint32_t B3;
  volatile const uint32_t B4;
  volatile const uint32_t B5;
  volatile const uint32_t T0;
  volatile const uint32_t T1;
  volatile const uint32_t T2;
  volatile const uint32_t T3;
  volatile const uint32_t T4;
} FICR_TEMP_Type;





typedef struct {
  volatile const uint32_t TAGHEADER0;


  volatile const uint32_t TAGHEADER1;


  volatile const uint32_t TAGHEADER2;


  volatile const uint32_t TAGHEADER3;


} FICR_NFC_Type;





typedef struct {
  volatile uint32_t POWER;
  volatile uint32_t POWERSET;

  volatile uint32_t POWERCLR;

  volatile const uint32_t RESERVED;
} POWER_RAM_Type;





typedef struct {
  volatile uint32_t RTS;
  volatile uint32_t TXD;
  volatile uint32_t CTS;
  volatile uint32_t RXD;
} UARTE_PSEL_Type;





typedef struct {
  volatile uint32_t PTR;
  volatile uint32_t MAXCNT;
  volatile const uint32_t AMOUNT;
} UARTE_RXD_Type;





typedef struct {
  volatile uint32_t PTR;
  volatile uint32_t MAXCNT;
  volatile const uint32_t AMOUNT;
} UARTE_TXD_Type;





typedef struct {
  volatile uint32_t SCK;
  volatile uint32_t MOSI;
  volatile uint32_t MISO;
} SPIM_PSEL_Type;





typedef struct {
  volatile uint32_t PTR;
  volatile uint32_t MAXCNT;
  volatile const uint32_t AMOUNT;
  volatile uint32_t LIST;
} SPIM_RXD_Type;





typedef struct {
  volatile uint32_t PTR;
  volatile uint32_t MAXCNT;
  volatile const uint32_t AMOUNT;
  volatile uint32_t LIST;
} SPIM_TXD_Type;





typedef struct {
  volatile uint32_t SCK;
  volatile uint32_t MISO;
  volatile uint32_t MOSI;
  volatile uint32_t CSN;
} SPIS_PSEL_Type;





typedef struct {
  volatile uint32_t PTR;
  volatile uint32_t MAXCNT;
  volatile const uint32_t AMOUNT;
} SPIS_RXD_Type;





typedef struct {
  volatile uint32_t PTR;
  volatile uint32_t MAXCNT;
  volatile const uint32_t AMOUNT;
} SPIS_TXD_Type;





typedef struct {
  volatile uint32_t SCL;
  volatile uint32_t SDA;
} TWIM_PSEL_Type;





typedef struct {
  volatile uint32_t PTR;
  volatile uint32_t MAXCNT;
  volatile const uint32_t AMOUNT;
  volatile uint32_t LIST;
} TWIM_RXD_Type;





typedef struct {
  volatile uint32_t PTR;
  volatile uint32_t MAXCNT;
  volatile const uint32_t AMOUNT;
  volatile uint32_t LIST;
} TWIM_TXD_Type;





typedef struct {
  volatile uint32_t SCL;
  volatile uint32_t SDA;
} TWIS_PSEL_Type;





typedef struct {
  volatile uint32_t PTR;
  volatile uint32_t MAXCNT;
  volatile const uint32_t AMOUNT;
} TWIS_RXD_Type;





typedef struct {
  volatile uint32_t PTR;
  volatile uint32_t MAXCNT;
  volatile const uint32_t AMOUNT;
} TWIS_TXD_Type;





typedef struct {
  volatile uint32_t SCK;
  volatile uint32_t MOSI;
  volatile uint32_t MISO;
} SPI_PSEL_Type;





typedef struct {
  volatile uint32_t RX;
} NFCT_FRAMESTATUS_Type;





typedef struct {
  volatile uint32_t FRAMECONFIG;
  volatile uint32_t AMOUNT;
} NFCT_TXD_Type;





typedef struct {
  volatile uint32_t FRAMECONFIG;
  volatile const uint32_t AMOUNT;
} NFCT_RXD_Type;





typedef struct {
  volatile uint32_t LIMITH;

  volatile uint32_t LIMITL;

} SAADC_EVENTS_CH_Type;





typedef struct {
  volatile uint32_t PSELP;

  volatile uint32_t PSELN;

  volatile uint32_t CONFIG;

  volatile uint32_t LIMIT;

} SAADC_CH_Type;





typedef struct {
  volatile uint32_t PTR;
  volatile uint32_t MAXCNT;
  volatile const uint32_t AMOUNT;

} SAADC_RESULT_Type;





typedef struct {
  volatile uint32_t LED;
  volatile uint32_t A;
  volatile uint32_t B;
} QDEC_PSEL_Type;





typedef struct {
  volatile uint32_t PTR;

  volatile uint32_t CNT;

  volatile uint32_t REFRESH;


  volatile uint32_t ENDDELAY;

  volatile const uint32_t RESERVED[4];
} PWM_SEQ_Type;





typedef struct {
  volatile uint32_t OUT[4];

} PWM_PSEL_Type;





typedef struct {
  volatile uint32_t CLK;
  volatile uint32_t DIN;
} PDM_PSEL_Type;





typedef struct {
  volatile uint32_t PTR;

  volatile uint32_t MAXCNT;

} PDM_SAMPLE_Type;





typedef struct {
  volatile uint32_t EN;

  volatile uint32_t DIS;

} PPI_TASKS_CHG_Type;





typedef struct {
  volatile uint32_t EEP;
  volatile uint32_t TEP;
} PPI_CH_Type;





typedef struct {
  volatile uint32_t TEP;
} PPI_FORK_Type;





typedef struct {
  volatile uint32_t WA;

  volatile uint32_t RA;

} MWU_EVENTS_REGION_Type;





typedef struct {
  volatile uint32_t WA;

  volatile uint32_t RA;

} MWU_EVENTS_PREGION_Type;





typedef struct {
  volatile uint32_t SUBSTATWA;



  volatile uint32_t SUBSTATRA;



} MWU_PERREGION_Type;





typedef struct {
  volatile uint32_t START;

  volatile uint32_t END;

  volatile const uint32_t RESERVED[2];
} MWU_REGION_Type;





typedef struct {
  volatile const uint32_t START;
  volatile const uint32_t END;
  volatile uint32_t SUBS;

  volatile const uint32_t RESERVED;
} MWU_PREGION_Type;





typedef struct {
  volatile uint32_t MODE;
  volatile uint32_t RXEN;
  volatile uint32_t TXEN;
  volatile uint32_t MCKEN;
  volatile uint32_t MCKFREQ;
  volatile uint32_t RATIO;
  volatile uint32_t SWIDTH;
  volatile uint32_t ALIGN;
  volatile uint32_t FORMAT;
  volatile uint32_t CHANNELS;
} I2S_CONFIG_Type;





typedef struct {
  volatile uint32_t PTR;
} I2S_RXD_Type;





typedef struct {
  volatile uint32_t PTR;
} I2S_TXD_Type;





typedef struct {
  volatile uint32_t MAXCNT;
} I2S_RXTXD_Type;





typedef struct {
  volatile uint32_t MCK;
  volatile uint32_t SCK;
  volatile uint32_t LRCK;
  volatile uint32_t SDIN;
  volatile uint32_t SDOUT;
} I2S_PSEL_Type;
# 717 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile const uint32_t RESERVED[4];
  volatile const uint32_t CODEPAGESIZE;
  volatile const uint32_t CODESIZE;
  volatile const uint32_t RESERVED1[18];
  volatile const uint32_t DEVICEID[2];
  volatile const uint32_t RESERVED2[6];
  volatile const uint32_t ER[4];

  volatile const uint32_t IR[4];

  volatile const uint32_t DEVICEADDRTYPE;
  volatile const uint32_t DEVICEADDR[2];
  volatile const uint32_t RESERVED3[21];
  volatile FICR_INFO_Type INFO;
  volatile const uint32_t RESERVED4[185];
  volatile FICR_TEMP_Type TEMP;

  volatile const uint32_t RESERVED5[2];
  volatile FICR_NFC_Type NFC;
} NRF_FICR_Type;
# 750 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t UNUSED0;
  volatile uint32_t UNUSED1;
  volatile uint32_t UNUSED2;
  volatile const uint32_t RESERVED;
  volatile uint32_t UNUSED3;
  volatile uint32_t NRFFW[15];

  volatile uint32_t NRFHW[12];

  volatile uint32_t CUSTOMER[32];
  volatile const uint32_t RESERVED1[64];
  volatile uint32_t PSELRESET[2];

  volatile uint32_t APPROTECT;
  volatile uint32_t NFCPINS;

} NRF_UICR_Type;
# 780 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile const uint32_t RESERVED[340];
  volatile uint32_t FORCEPROTECT;


  volatile const uint32_t RESERVED1;
  volatile uint32_t DISABLE;
} NRF_APPROTECT_Type;
# 800 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile const uint32_t RESERVED[384];
  volatile uint32_t CONFIG0;
  volatile uint32_t CONFIG1;
  volatile uint32_t DISABLEINDEBUG;

  volatile uint32_t UNUSED0;
  volatile uint32_t CONFIG2;
  volatile uint32_t CONFIG3;
} NRF_BPROT_Type;
# 822 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile const uint32_t RESERVED[30];
  volatile uint32_t TASKS_CONSTLAT;
  volatile uint32_t TASKS_LOWPWR;
  volatile const uint32_t RESERVED1[34];
  volatile uint32_t EVENTS_POFWARN;
  volatile const uint32_t RESERVED2[2];
  volatile uint32_t EVENTS_SLEEPENTER;
  volatile uint32_t EVENTS_SLEEPEXIT;
  volatile const uint32_t RESERVED3[122];
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED4[61];
  volatile uint32_t RESETREAS;
  volatile const uint32_t RESERVED5[9];
  volatile const uint32_t RAMSTATUS;
  volatile const uint32_t RESERVED6[53];
  volatile uint32_t SYSTEMOFF;
  volatile const uint32_t RESERVED7[3];
  volatile uint32_t POFCON;
  volatile const uint32_t RESERVED8[2];
  volatile uint32_t GPREGRET;
  volatile uint32_t GPREGRET2;
  volatile uint32_t RAMON;

  volatile const uint32_t RESERVED9[11];
  volatile uint32_t RAMONB;

  volatile const uint32_t RESERVED10[8];
  volatile uint32_t DCDCEN;
  volatile const uint32_t RESERVED11[225];
  volatile POWER_RAM_Type RAM[8];
} NRF_POWER_Type;
# 867 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_HFCLKSTART;
  volatile uint32_t TASKS_HFCLKSTOP;
  volatile uint32_t TASKS_LFCLKSTART;
  volatile uint32_t TASKS_LFCLKSTOP;
  volatile uint32_t TASKS_CAL;
  volatile uint32_t TASKS_CTSTART;
  volatile uint32_t TASKS_CTSTOP;
  volatile const uint32_t RESERVED[57];
  volatile uint32_t EVENTS_HFCLKSTARTED;
  volatile uint32_t EVENTS_LFCLKSTARTED;
  volatile const uint32_t RESERVED1;
  volatile uint32_t EVENTS_DONE;
  volatile uint32_t EVENTS_CTTO;
  volatile const uint32_t RESERVED2[124];
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED3[63];
  volatile const uint32_t HFCLKRUN;

  volatile const uint32_t HFCLKSTAT;
  volatile const uint32_t RESERVED4;
  volatile const uint32_t LFCLKRUN;

  volatile const uint32_t LFCLKSTAT;
  volatile const uint32_t LFCLKSRCCOPY;

  volatile const uint32_t RESERVED5[62];
  volatile uint32_t LFCLKSRC;
  volatile const uint32_t RESERVED6[7];
  volatile uint32_t CTIV;
  volatile const uint32_t RESERVED7[8];
  volatile uint32_t TRACECONFIG;
} NRF_CLOCK_Type;
# 913 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_TXEN;
  volatile uint32_t TASKS_RXEN;
  volatile uint32_t TASKS_START;
  volatile uint32_t TASKS_STOP;
  volatile uint32_t TASKS_DISABLE;
  volatile uint32_t TASKS_RSSISTART;

  volatile uint32_t TASKS_RSSISTOP;
  volatile uint32_t TASKS_BCSTART;
  volatile uint32_t TASKS_BCSTOP;
  volatile const uint32_t RESERVED[55];
  volatile uint32_t EVENTS_READY;
  volatile uint32_t EVENTS_ADDRESS;
  volatile uint32_t EVENTS_PAYLOAD;
  volatile uint32_t EVENTS_END;
  volatile uint32_t EVENTS_DISABLED;
  volatile uint32_t EVENTS_DEVMATCH;

  volatile uint32_t EVENTS_DEVMISS;

  volatile uint32_t EVENTS_RSSIEND;
  volatile const uint32_t RESERVED1[2];
  volatile uint32_t EVENTS_BCMATCH;
  volatile const uint32_t RESERVED2;
  volatile uint32_t EVENTS_CRCOK;
  volatile uint32_t EVENTS_CRCERROR;
  volatile const uint32_t RESERVED3[50];
  volatile uint32_t SHORTS;
  volatile const uint32_t RESERVED4[64];
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED5[61];
  volatile const uint32_t CRCSTATUS;
  volatile const uint32_t RESERVED6;
  volatile const uint32_t RXMATCH;
  volatile const uint32_t RXCRC;
  volatile const uint32_t DAI;
  volatile const uint32_t RESERVED7[60];
  volatile uint32_t PACKETPTR;
  volatile uint32_t FREQUENCY;
  volatile uint32_t TXPOWER;
  volatile uint32_t MODE;
  volatile uint32_t PCNF0;
  volatile uint32_t PCNF1;
  volatile uint32_t BASE0;
  volatile uint32_t BASE1;
  volatile uint32_t PREFIX0;
  volatile uint32_t PREFIX1;
  volatile uint32_t TXADDRESS;
  volatile uint32_t RXADDRESSES;
  volatile uint32_t CRCCNF;
  volatile uint32_t CRCPOLY;
  volatile uint32_t CRCINIT;
  volatile uint32_t UNUSED0;
  volatile uint32_t TIFS;
  volatile const uint32_t RSSISAMPLE;
  volatile const uint32_t RESERVED8;
  volatile const uint32_t STATE;
  volatile uint32_t DATAWHITEIV;
  volatile const uint32_t RESERVED9[2];
  volatile uint32_t BCC;
  volatile const uint32_t RESERVED10[39];
  volatile uint32_t DAB[8];

  volatile uint32_t DAP[8];

  volatile uint32_t DACNF;
  volatile const uint32_t RESERVED11[3];
  volatile uint32_t MODECNF0;
  volatile const uint32_t RESERVED12[618];
  volatile uint32_t POWER;
} NRF_RADIO_Type;
# 998 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_STARTRX;
  volatile uint32_t TASKS_STOPRX;
  volatile uint32_t TASKS_STARTTX;
  volatile uint32_t TASKS_STOPTX;
  volatile const uint32_t RESERVED[7];
  volatile uint32_t TASKS_FLUSHRX;
  volatile const uint32_t RESERVED1[52];
  volatile uint32_t EVENTS_CTS;
  volatile uint32_t EVENTS_NCTS;
  volatile uint32_t EVENTS_RXDRDY;

  volatile const uint32_t RESERVED2;
  volatile uint32_t EVENTS_ENDRX;
  volatile const uint32_t RESERVED3[2];
  volatile uint32_t EVENTS_TXDRDY;
  volatile uint32_t EVENTS_ENDTX;
  volatile uint32_t EVENTS_ERROR;
  volatile const uint32_t RESERVED4[7];
  volatile uint32_t EVENTS_RXTO;
  volatile const uint32_t RESERVED5;
  volatile uint32_t EVENTS_RXSTARTED;
  volatile uint32_t EVENTS_TXSTARTED;
  volatile const uint32_t RESERVED6;
  volatile uint32_t EVENTS_TXSTOPPED;
  volatile const uint32_t RESERVED7[41];
  volatile uint32_t SHORTS;
  volatile const uint32_t RESERVED8[63];
  volatile uint32_t INTEN;
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED9[93];
  volatile uint32_t ERRORSRC;
  volatile const uint32_t RESERVED10[31];
  volatile uint32_t ENABLE;
  volatile const uint32_t RESERVED11;
  volatile UARTE_PSEL_Type PSEL;
  volatile const uint32_t RESERVED12[3];
  volatile uint32_t BAUDRATE;

  volatile const uint32_t RESERVED13[3];
  volatile UARTE_RXD_Type RXD;
  volatile const uint32_t RESERVED14;
  volatile UARTE_TXD_Type TXD;
  volatile const uint32_t RESERVED15[7];
  volatile uint32_t CONFIG;
} NRF_UARTE_Type;
# 1057 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_STARTRX;
  volatile uint32_t TASKS_STOPRX;
  volatile uint32_t TASKS_STARTTX;
  volatile uint32_t TASKS_STOPTX;
  volatile const uint32_t RESERVED[3];
  volatile uint32_t TASKS_SUSPEND;
  volatile const uint32_t RESERVED1[56];
  volatile uint32_t EVENTS_CTS;
  volatile uint32_t EVENTS_NCTS;
  volatile uint32_t EVENTS_RXDRDY;
  volatile const uint32_t RESERVED2[4];
  volatile uint32_t EVENTS_TXDRDY;
  volatile const uint32_t RESERVED3;
  volatile uint32_t EVENTS_ERROR;
  volatile const uint32_t RESERVED4[7];
  volatile uint32_t EVENTS_RXTO;
  volatile const uint32_t RESERVED5[46];
  volatile uint32_t SHORTS;
  volatile const uint32_t RESERVED6[64];
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED7[93];
  volatile uint32_t ERRORSRC;
  volatile const uint32_t RESERVED8[31];
  volatile uint32_t ENABLE;
  volatile const uint32_t RESERVED9;
  volatile uint32_t PSELRTS;
  volatile uint32_t PSELTXD;
  volatile uint32_t PSELCTS;
  volatile uint32_t PSELRXD;
  volatile const uint32_t RXD;
  volatile uint32_t TXD;
  volatile const uint32_t RESERVED10;
  volatile uint32_t BAUDRATE;
  volatile const uint32_t RESERVED11[17];
  volatile uint32_t CONFIG;
} NRF_UART_Type;
# 1107 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile const uint32_t RESERVED[4];
  volatile uint32_t TASKS_START;
  volatile uint32_t TASKS_STOP;
  volatile const uint32_t RESERVED1;
  volatile uint32_t TASKS_SUSPEND;
  volatile uint32_t TASKS_RESUME;
  volatile const uint32_t RESERVED2[56];
  volatile uint32_t EVENTS_STOPPED;
  volatile const uint32_t RESERVED3[2];
  volatile uint32_t EVENTS_ENDRX;
  volatile const uint32_t RESERVED4;
  volatile uint32_t EVENTS_END;
  volatile const uint32_t RESERVED5;
  volatile uint32_t EVENTS_ENDTX;
  volatile const uint32_t RESERVED6[10];
  volatile uint32_t EVENTS_STARTED;
  volatile const uint32_t RESERVED7[44];
  volatile uint32_t SHORTS;
  volatile const uint32_t RESERVED8[64];
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED9[125];
  volatile uint32_t ENABLE;
  volatile const uint32_t RESERVED10;
  volatile SPIM_PSEL_Type PSEL;
  volatile const uint32_t RESERVED11[4];
  volatile uint32_t FREQUENCY;

  volatile const uint32_t RESERVED12[3];
  volatile SPIM_RXD_Type RXD;
  volatile SPIM_TXD_Type TXD;
  volatile uint32_t CONFIG;
  volatile const uint32_t RESERVED13[26];
  volatile uint32_t ORC;

} NRF_SPIM_Type;
# 1156 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile const uint32_t RESERVED[9];
  volatile uint32_t TASKS_ACQUIRE;
  volatile uint32_t TASKS_RELEASE;

  volatile const uint32_t RESERVED1[54];
  volatile uint32_t EVENTS_END;
  volatile const uint32_t RESERVED2[2];
  volatile uint32_t EVENTS_ENDRX;
  volatile const uint32_t RESERVED3[5];
  volatile uint32_t EVENTS_ACQUIRED;
  volatile const uint32_t RESERVED4[53];
  volatile uint32_t SHORTS;
  volatile const uint32_t RESERVED5[64];
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED6[61];
  volatile const uint32_t SEMSTAT;
  volatile const uint32_t RESERVED7[15];
  volatile uint32_t STATUS;
  volatile const uint32_t RESERVED8[47];
  volatile uint32_t ENABLE;
  volatile const uint32_t RESERVED9;
  volatile SPIS_PSEL_Type PSEL;
  volatile const uint32_t RESERVED10[7];
  volatile SPIS_RXD_Type RXD;
  volatile const uint32_t RESERVED11;
  volatile SPIS_TXD_Type TXD;
  volatile const uint32_t RESERVED12;
  volatile uint32_t CONFIG;
  volatile const uint32_t RESERVED13;
  volatile uint32_t DEF;

  volatile const uint32_t RESERVED14[24];
  volatile uint32_t ORC;
} NRF_SPIS_Type;
# 1204 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_STARTRX;
  volatile const uint32_t RESERVED;
  volatile uint32_t TASKS_STARTTX;
  volatile const uint32_t RESERVED1[2];
  volatile uint32_t TASKS_STOP;

  volatile const uint32_t RESERVED2;
  volatile uint32_t TASKS_SUSPEND;
  volatile uint32_t TASKS_RESUME;
  volatile const uint32_t RESERVED3[56];
  volatile uint32_t EVENTS_STOPPED;
  volatile const uint32_t RESERVED4[7];
  volatile uint32_t EVENTS_ERROR;
  volatile const uint32_t RESERVED5[8];
  volatile uint32_t EVENTS_SUSPENDED;


  volatile uint32_t EVENTS_RXSTARTED;
  volatile uint32_t EVENTS_TXSTARTED;
  volatile const uint32_t RESERVED6[2];
  volatile uint32_t EVENTS_LASTRX;
  volatile uint32_t EVENTS_LASTTX;

  volatile const uint32_t RESERVED7[39];
  volatile uint32_t SHORTS;
  volatile const uint32_t RESERVED8[63];
  volatile uint32_t INTEN;
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED9[110];
  volatile uint32_t ERRORSRC;
  volatile const uint32_t RESERVED10[14];
  volatile uint32_t ENABLE;
  volatile const uint32_t RESERVED11;
  volatile TWIM_PSEL_Type PSEL;
  volatile const uint32_t RESERVED12[5];
  volatile uint32_t FREQUENCY;
  volatile const uint32_t RESERVED13[3];
  volatile TWIM_RXD_Type RXD;
  volatile TWIM_TXD_Type TXD;
  volatile const uint32_t RESERVED14[13];
  volatile uint32_t ADDRESS;
} NRF_TWIM_Type;
# 1260 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile const uint32_t RESERVED[5];
  volatile uint32_t TASKS_STOP;
  volatile const uint32_t RESERVED1;
  volatile uint32_t TASKS_SUSPEND;
  volatile uint32_t TASKS_RESUME;
  volatile const uint32_t RESERVED2[3];
  volatile uint32_t TASKS_PREPARERX;
  volatile uint32_t TASKS_PREPARETX;
  volatile const uint32_t RESERVED3[51];
  volatile uint32_t EVENTS_STOPPED;
  volatile const uint32_t RESERVED4[7];
  volatile uint32_t EVENTS_ERROR;
  volatile const uint32_t RESERVED5[9];
  volatile uint32_t EVENTS_RXSTARTED;
  volatile uint32_t EVENTS_TXSTARTED;
  volatile const uint32_t RESERVED6[4];
  volatile uint32_t EVENTS_WRITE;
  volatile uint32_t EVENTS_READ;
  volatile const uint32_t RESERVED7[37];
  volatile uint32_t SHORTS;
  volatile const uint32_t RESERVED8[63];
  volatile uint32_t INTEN;
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED9[113];
  volatile uint32_t ERRORSRC;
  volatile const uint32_t MATCH;

  volatile const uint32_t RESERVED10[10];
  volatile uint32_t ENABLE;
  volatile const uint32_t RESERVED11;
  volatile TWIS_PSEL_Type PSEL;
  volatile const uint32_t RESERVED12[9];
  volatile TWIS_RXD_Type RXD;
  volatile const uint32_t RESERVED13;
  volatile TWIS_TXD_Type TXD;
  volatile const uint32_t RESERVED14[14];
  volatile uint32_t ADDRESS[2];

  volatile const uint32_t RESERVED15;
  volatile uint32_t CONFIG;

  volatile const uint32_t RESERVED16[10];
  volatile uint32_t ORC;

} NRF_TWIS_Type;
# 1319 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile const uint32_t RESERVED[66];
  volatile uint32_t EVENTS_READY;
  volatile const uint32_t RESERVED1[126];
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED2[125];
  volatile uint32_t ENABLE;
  volatile const uint32_t RESERVED3;
  volatile SPI_PSEL_Type PSEL;
  volatile const uint32_t RESERVED4;
  volatile const uint32_t RXD;
  volatile uint32_t TXD;
  volatile const uint32_t RESERVED5;
  volatile uint32_t FREQUENCY;
  volatile const uint32_t RESERVED6[11];
  volatile uint32_t CONFIG;
} NRF_SPI_Type;
# 1349 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_STARTRX;
  volatile const uint32_t RESERVED;
  volatile uint32_t TASKS_STARTTX;
  volatile const uint32_t RESERVED1[2];
  volatile uint32_t TASKS_STOP;
  volatile const uint32_t RESERVED2;
  volatile uint32_t TASKS_SUSPEND;
  volatile uint32_t TASKS_RESUME;
  volatile const uint32_t RESERVED3[56];
  volatile uint32_t EVENTS_STOPPED;
  volatile uint32_t EVENTS_RXDREADY;
  volatile const uint32_t RESERVED4[4];
  volatile uint32_t EVENTS_TXDSENT;
  volatile const uint32_t RESERVED5;
  volatile uint32_t EVENTS_ERROR;
  volatile const uint32_t RESERVED6[4];
  volatile uint32_t EVENTS_BB;

  volatile const uint32_t RESERVED7[3];
  volatile uint32_t EVENTS_SUSPENDED;
  volatile const uint32_t RESERVED8[45];
  volatile uint32_t SHORTS;
  volatile const uint32_t RESERVED9[64];
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED10[110];
  volatile uint32_t ERRORSRC;
  volatile const uint32_t RESERVED11[14];
  volatile uint32_t ENABLE;
  volatile const uint32_t RESERVED12;
  volatile uint32_t PSELSCL;
  volatile uint32_t PSELSDA;
  volatile const uint32_t RESERVED13[2];
  volatile const uint32_t RXD;
  volatile uint32_t TXD;
  volatile const uint32_t RESERVED14;
  volatile uint32_t FREQUENCY;
  volatile const uint32_t RESERVED15[24];
  volatile uint32_t ADDRESS;
} NRF_TWI_Type;
# 1402 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_ACTIVATE;

  volatile uint32_t TASKS_DISABLE;
  volatile uint32_t TASKS_SENSE;

  volatile uint32_t TASKS_STARTTX;

  volatile const uint32_t RESERVED[3];
  volatile uint32_t TASKS_ENABLERXDATA;
  volatile const uint32_t RESERVED1;
  volatile uint32_t TASKS_GOIDLE;
  volatile uint32_t TASKS_GOSLEEP;
  volatile const uint32_t RESERVED2[53];
  volatile uint32_t EVENTS_READY;

  volatile uint32_t EVENTS_FIELDDETECTED;
  volatile uint32_t EVENTS_FIELDLOST;
  volatile uint32_t EVENTS_TXFRAMESTART;

  volatile uint32_t EVENTS_TXFRAMEEND;

  volatile uint32_t EVENTS_RXFRAMESTART;

  volatile uint32_t EVENTS_RXFRAMEEND;


  volatile uint32_t EVENTS_ERROR;

  volatile const uint32_t RESERVED3[2];
  volatile uint32_t EVENTS_RXERROR;


  volatile uint32_t EVENTS_ENDRX;

  volatile uint32_t EVENTS_ENDTX;

  volatile const uint32_t RESERVED4;
  volatile uint32_t EVENTS_AUTOCOLRESSTARTED;
  volatile const uint32_t RESERVED5[3];
  volatile uint32_t EVENTS_COLLISION;
  volatile uint32_t EVENTS_SELECTED;
  volatile uint32_t EVENTS_STARTED;
  volatile const uint32_t RESERVED6[43];
  volatile uint32_t SHORTS;
  volatile const uint32_t RESERVED7[63];
  volatile uint32_t INTEN;
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED8[62];
  volatile uint32_t ERRORSTATUS;
  volatile const uint32_t RESERVED9;
  volatile NFCT_FRAMESTATUS_Type FRAMESTATUS;
  volatile const uint32_t RESERVED10[8];
  volatile const uint32_t CURRENTLOADCTRL;
  volatile const uint32_t RESERVED11[2];
  volatile const uint32_t FIELDPRESENT;
  volatile const uint32_t RESERVED12[49];
  volatile uint32_t FRAMEDELAYMIN;
  volatile uint32_t FRAMEDELAYMAX;
  volatile uint32_t FRAMEDELAYMODE;
  volatile uint32_t PACKETPTR;

  volatile uint32_t MAXLEN;

  volatile NFCT_TXD_Type TXD;
  volatile NFCT_RXD_Type RXD;
  volatile const uint32_t RESERVED13[26];
  volatile uint32_t NFCID1_LAST;
  volatile uint32_t NFCID1_2ND_LAST;
  volatile uint32_t NFCID1_3RD_LAST;
  volatile const uint32_t RESERVED14;
  volatile uint32_t SENSRES;
  volatile uint32_t SELRES;
} NRF_NFCT_Type;
# 1489 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_OUT[8];


  volatile const uint32_t RESERVED[4];
  volatile uint32_t TASKS_SET[8];


  volatile const uint32_t RESERVED1[4];
  volatile uint32_t TASKS_CLR[8];


  volatile const uint32_t RESERVED2[32];
  volatile uint32_t EVENTS_IN[8];

  volatile const uint32_t RESERVED3[23];
  volatile uint32_t EVENTS_PORT;

  volatile const uint32_t RESERVED4[97];
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED5[129];
  volatile uint32_t CONFIG[8];


} NRF_GPIOTE_Type;
# 1527 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_START;

  volatile uint32_t TASKS_SAMPLE;

  volatile uint32_t TASKS_STOP;
  volatile uint32_t TASKS_CALIBRATEOFFSET;
  volatile const uint32_t RESERVED[60];
  volatile uint32_t EVENTS_STARTED;
  volatile uint32_t EVENTS_END;
  volatile uint32_t EVENTS_DONE;



  volatile uint32_t EVENTS_RESULTDONE;
  volatile uint32_t EVENTS_CALIBRATEDONE;
  volatile uint32_t EVENTS_STOPPED;
  volatile SAADC_EVENTS_CH_Type EVENTS_CH[8];
  volatile const uint32_t RESERVED1[106];
  volatile uint32_t INTEN;
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED2[61];
  volatile const uint32_t STATUS;
  volatile const uint32_t RESERVED3[63];
  volatile uint32_t ENABLE;
  volatile const uint32_t RESERVED4[3];
  volatile SAADC_CH_Type CH[8];
  volatile const uint32_t RESERVED5[24];
  volatile uint32_t RESOLUTION;
  volatile uint32_t OVERSAMPLE;




  volatile uint32_t SAMPLERATE;
  volatile const uint32_t RESERVED6[12];
  volatile SAADC_RESULT_Type RESULT;
} NRF_SAADC_Type;
# 1578 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_START;
  volatile uint32_t TASKS_STOP;
  volatile uint32_t TASKS_COUNT;
  volatile uint32_t TASKS_CLEAR;
  volatile uint32_t TASKS_SHUTDOWN;
  volatile const uint32_t RESERVED[11];
  volatile uint32_t TASKS_CAPTURE[6];

  volatile const uint32_t RESERVED1[58];
  volatile uint32_t EVENTS_COMPARE[6];

  volatile const uint32_t RESERVED2[42];
  volatile uint32_t SHORTS;
  volatile const uint32_t RESERVED3[64];
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED4[126];
  volatile uint32_t MODE;
  volatile uint32_t BITMODE;
  volatile const uint32_t RESERVED5;
  volatile uint32_t PRESCALER;
  volatile const uint32_t RESERVED6[11];
  volatile uint32_t CC[6];

} NRF_TIMER_Type;
# 1616 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_START;
  volatile uint32_t TASKS_STOP;
  volatile uint32_t TASKS_CLEAR;
  volatile uint32_t TASKS_TRIGOVRFLW;
  volatile const uint32_t RESERVED[60];
  volatile uint32_t EVENTS_TICK;
  volatile uint32_t EVENTS_OVRFLW;
  volatile const uint32_t RESERVED1[14];
  volatile uint32_t EVENTS_COMPARE[4];

  volatile const uint32_t RESERVED2[109];
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED3[13];
  volatile uint32_t EVTEN;
  volatile uint32_t EVTENSET;
  volatile uint32_t EVTENCLR;
  volatile const uint32_t RESERVED4[110];
  volatile const uint32_t COUNTER;
  volatile uint32_t PRESCALER;

  volatile const uint32_t RESERVED5[13];
  volatile uint32_t CC[4];
} NRF_RTC_Type;
# 1653 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_START;
  volatile uint32_t TASKS_STOP;
  volatile const uint32_t RESERVED[62];
  volatile uint32_t EVENTS_DATARDY;
  volatile const uint32_t RESERVED1[128];
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED2[127];
  volatile const int32_t TEMP;
  volatile const uint32_t RESERVED3[5];
  volatile uint32_t A0;
  volatile uint32_t A1;
  volatile uint32_t A2;
  volatile uint32_t A3;
  volatile uint32_t A4;
  volatile uint32_t A5;
  volatile const uint32_t RESERVED4[2];
  volatile uint32_t B0;
  volatile uint32_t B1;
  volatile uint32_t B2;
  volatile uint32_t B3;
  volatile uint32_t B4;
  volatile uint32_t B5;
  volatile const uint32_t RESERVED5[2];
  volatile uint32_t T0;
  volatile uint32_t T1;
  volatile uint32_t T2;
  volatile uint32_t T3;
  volatile uint32_t T4;
} NRF_TEMP_Type;
# 1696 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_START;
  volatile uint32_t TASKS_STOP;
  volatile const uint32_t RESERVED[62];
  volatile uint32_t EVENTS_VALRDY;

  volatile const uint32_t RESERVED1[63];
  volatile uint32_t SHORTS;
  volatile const uint32_t RESERVED2[64];
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED3[126];
  volatile uint32_t CONFIG;
  volatile const uint32_t VALUE;
} NRF_RNG_Type;
# 1723 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_STARTECB;
  volatile uint32_t TASKS_STOPECB;
  volatile const uint32_t RESERVED[62];
  volatile uint32_t EVENTS_ENDECB;
  volatile uint32_t EVENTS_ERRORECB;

  volatile const uint32_t RESERVED1[127];
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED2[126];
  volatile uint32_t ECBDATAPTR;
} NRF_ECB_Type;
# 1748 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_KSGEN;

  volatile uint32_t TASKS_CRYPT;

  volatile uint32_t TASKS_STOP;
  volatile const uint32_t RESERVED[61];
  volatile uint32_t EVENTS_ENDKSGEN;
  volatile uint32_t EVENTS_ENDCRYPT;
  volatile uint32_t EVENTS_ERROR;
  volatile const uint32_t RESERVED1[61];
  volatile uint32_t SHORTS;
  volatile const uint32_t RESERVED2[64];
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED3[61];
  volatile const uint32_t MICSTATUS;
  volatile const uint32_t RESERVED4[63];
  volatile uint32_t ENABLE;
  volatile uint32_t MODE;
  volatile uint32_t CNFPTR;

  volatile uint32_t INPTR;
  volatile uint32_t OUTPTR;
  volatile uint32_t SCRATCHPTR;
} NRF_CCM_Type;
# 1786 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_START;

  volatile const uint32_t RESERVED;
  volatile uint32_t TASKS_STOP;
  volatile const uint32_t RESERVED1[61];
  volatile uint32_t EVENTS_END;
  volatile uint32_t EVENTS_RESOLVED;
  volatile uint32_t EVENTS_NOTRESOLVED;
  volatile const uint32_t RESERVED2[126];
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED3[61];
  volatile const uint32_t STATUS;
  volatile const uint32_t RESERVED4[63];
  volatile uint32_t ENABLE;
  volatile uint32_t NIRK;
  volatile uint32_t IRKPTR;
  volatile const uint32_t RESERVED5;
  volatile uint32_t ADDRPTR;
  volatile uint32_t SCRATCHPTR;
} NRF_AAR_Type;
# 1820 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_START;
  volatile const uint32_t RESERVED[63];
  volatile uint32_t EVENTS_TIMEOUT;
  volatile const uint32_t RESERVED1[128];
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED2[61];
  volatile const uint32_t RUNSTATUS;
  volatile const uint32_t REQSTATUS;
  volatile const uint32_t RESERVED3[63];
  volatile uint32_t CRV;
  volatile uint32_t RREN;
  volatile uint32_t CONFIG;
  volatile const uint32_t RESERVED4[60];
  volatile uint32_t RR[8];
} NRF_WDT_Type;
# 1849 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_START;
  volatile uint32_t TASKS_STOP;
  volatile uint32_t TASKS_READCLRACC;
  volatile uint32_t TASKS_RDCLRACC;
  volatile uint32_t TASKS_RDCLRDBL;
  volatile const uint32_t RESERVED[59];
  volatile uint32_t EVENTS_SAMPLERDY;

  volatile uint32_t EVENTS_REPORTRDY;
  volatile uint32_t EVENTS_ACCOF;
  volatile uint32_t EVENTS_DBLRDY;
  volatile uint32_t EVENTS_STOPPED;
  volatile const uint32_t RESERVED1[59];
  volatile uint32_t SHORTS;
  volatile const uint32_t RESERVED2[64];
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED3[125];
  volatile uint32_t ENABLE;
  volatile uint32_t LEDPOL;
  volatile uint32_t SAMPLEPER;
  volatile const int32_t SAMPLE;
  volatile uint32_t REPORTPER;

  volatile const int32_t ACC;
  volatile const int32_t ACCREAD;

  volatile QDEC_PSEL_Type PSEL;
  volatile uint32_t DBFEN;
  volatile const uint32_t RESERVED4[5];
  volatile uint32_t LEDPRE;
  volatile const uint32_t ACCDBL;

  volatile const uint32_t ACCDBLREAD;

} NRF_QDEC_Type;
# 1898 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_START;
  volatile uint32_t TASKS_STOP;
  volatile uint32_t TASKS_SAMPLE;
  volatile const uint32_t RESERVED[61];
  volatile uint32_t EVENTS_READY;
  volatile uint32_t EVENTS_DOWN;
  volatile uint32_t EVENTS_UP;
  volatile uint32_t EVENTS_CROSS;
  volatile const uint32_t RESERVED1[60];
  volatile uint32_t SHORTS;
  volatile const uint32_t RESERVED2[63];
  volatile uint32_t INTEN;
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED3[61];
  volatile const uint32_t RESULT;
  volatile const uint32_t RESERVED4[63];
  volatile uint32_t ENABLE;
  volatile uint32_t PSEL;
  volatile uint32_t REFSEL;
  volatile uint32_t EXTREFSEL;
  volatile const uint32_t RESERVED5[8];
  volatile uint32_t TH;
  volatile uint32_t MODE;
  volatile uint32_t HYST;
  volatile uint32_t ISOURCE;
} NRF_COMP_Type;
# 1938 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_START;
  volatile uint32_t TASKS_STOP;
  volatile uint32_t TASKS_SAMPLE;
  volatile const uint32_t RESERVED[61];
  volatile uint32_t EVENTS_READY;
  volatile uint32_t EVENTS_DOWN;
  volatile uint32_t EVENTS_UP;
  volatile uint32_t EVENTS_CROSS;
  volatile const uint32_t RESERVED1[60];
  volatile uint32_t SHORTS;
  volatile const uint32_t RESERVED2[64];
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED3[61];
  volatile const uint32_t RESULT;
  volatile const uint32_t RESERVED4[63];
  volatile uint32_t ENABLE;
  volatile uint32_t PSEL;
  volatile uint32_t REFSEL;
  volatile uint32_t EXTREFSEL;
  volatile const uint32_t RESERVED5[4];
  volatile uint32_t ANADETECT;
  volatile const uint32_t RESERVED6[5];
  volatile uint32_t HYST;
} NRF_LPCOMP_Type;
# 1976 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile const uint32_t UNUSED;
} NRF_SWI_Type;
# 1991 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_TRIGGER[16];

  volatile const uint32_t RESERVED[48];
  volatile uint32_t EVENTS_TRIGGERED[16];


  volatile const uint32_t RESERVED1[112];
  volatile uint32_t INTEN;
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
} NRF_EGU_Type;
# 2015 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile const uint32_t RESERVED;
  volatile uint32_t TASKS_STOP;


  volatile uint32_t TASKS_SEQSTART[2];





  volatile uint32_t TASKS_NEXTSTEP;



  volatile const uint32_t RESERVED1[60];
  volatile uint32_t EVENTS_STOPPED;

  volatile uint32_t EVENTS_SEQSTARTED[2];

  volatile uint32_t EVENTS_SEQEND[2];


  volatile uint32_t EVENTS_PWMPERIODEND;
  volatile uint32_t EVENTS_LOOPSDONE;

  volatile const uint32_t RESERVED2[56];
  volatile uint32_t SHORTS;
  volatile const uint32_t RESERVED3[63];
  volatile uint32_t INTEN;
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED4[125];
  volatile uint32_t ENABLE;
  volatile uint32_t MODE;
  volatile uint32_t COUNTERTOP;

  volatile uint32_t PRESCALER;
  volatile uint32_t DECODER;
  volatile uint32_t LOOP;
  volatile const uint32_t RESERVED5[2];
  volatile PWM_SEQ_Type SEQ[2];
  volatile PWM_PSEL_Type PSEL;
} NRF_PWM_Type;
# 2071 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_START;
  volatile uint32_t TASKS_STOP;
  volatile const uint32_t RESERVED[62];
  volatile uint32_t EVENTS_STARTED;
  volatile uint32_t EVENTS_STOPPED;
  volatile uint32_t EVENTS_END;


  volatile const uint32_t RESERVED1[125];
  volatile uint32_t INTEN;
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED2[125];
  volatile uint32_t ENABLE;
  volatile uint32_t PDMCLKCTRL;
  volatile uint32_t MODE;

  volatile const uint32_t RESERVED3[3];
  volatile uint32_t GAINL;
  volatile uint32_t GAINR;
  volatile const uint32_t RESERVED4[8];
  volatile PDM_PSEL_Type PSEL;
  volatile const uint32_t RESERVED5[6];
  volatile PDM_SAMPLE_Type SAMPLE;
} NRF_PDM_Type;
# 2109 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile const uint32_t RESERVED[256];
  volatile const uint32_t READY;
  volatile const uint32_t RESERVED1[64];
  volatile uint32_t CONFIG;

  union {
    volatile uint32_t ERASEPAGE;
    volatile uint32_t ERASEPCR1;

  };
  volatile uint32_t ERASEALL;
  volatile uint32_t ERASEPCR0;

  volatile uint32_t ERASEUICR;

  volatile const uint32_t RESERVED2[10];
  volatile uint32_t ICACHECNF;
  volatile const uint32_t RESERVED3;
  volatile uint32_t IHIT;
  volatile uint32_t IMISS;
} NRF_NVMC_Type;
# 2143 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile PPI_TASKS_CHG_Type TASKS_CHG[6];
  volatile const uint32_t RESERVED[308];
  volatile uint32_t CHEN;
  volatile uint32_t CHENSET;
  volatile uint32_t CHENCLR;
  volatile const uint32_t RESERVED1;
  volatile PPI_CH_Type CH[20];
  volatile const uint32_t RESERVED2[148];
  volatile uint32_t CHG[6];
  volatile const uint32_t RESERVED3[62];
  volatile PPI_FORK_Type FORK[32];
} NRF_PPI_Type;
# 2168 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile const uint32_t RESERVED[64];
  volatile MWU_EVENTS_REGION_Type EVENTS_REGION[4];
  volatile const uint32_t RESERVED1[16];
  volatile MWU_EVENTS_PREGION_Type EVENTS_PREGION[2];
  volatile const uint32_t RESERVED2[100];
  volatile uint32_t INTEN;
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED3[5];
  volatile uint32_t NMIEN;
  volatile uint32_t NMIENSET;
  volatile uint32_t NMIENCLR;
  volatile const uint32_t RESERVED4[53];
  volatile MWU_PERREGION_Type PERREGION[2];
  volatile const uint32_t RESERVED5[64];
  volatile uint32_t REGIONEN;
  volatile uint32_t REGIONENSET;
  volatile uint32_t REGIONENCLR;
  volatile const uint32_t RESERVED6[57];
  volatile MWU_REGION_Type REGION[4];
  volatile const uint32_t RESERVED7[32];
  volatile MWU_PREGION_Type PREGION[2];
} NRF_MWU_Type;
# 2204 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile uint32_t TASKS_START;

  volatile uint32_t TASKS_STOP;


  volatile const uint32_t RESERVED[63];
  volatile uint32_t EVENTS_RXPTRUPD;




  volatile uint32_t EVENTS_STOPPED;
  volatile const uint32_t RESERVED1[2];
  volatile uint32_t EVENTS_TXPTRUPD;




  volatile const uint32_t RESERVED2[122];
  volatile uint32_t INTEN;
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile const uint32_t RESERVED3[125];
  volatile uint32_t ENABLE;
  volatile I2S_CONFIG_Type CONFIG;
  volatile const uint32_t RESERVED4[3];
  volatile I2S_RXD_Type RXD;
  volatile const uint32_t RESERVED5;
  volatile I2S_TXD_Type TXD;
  volatile const uint32_t RESERVED6[3];
  volatile I2S_RXTXD_Type RXTXD;
  volatile const uint32_t RESERVED7[3];
  volatile I2S_PSEL_Type PSEL;
} NRF_I2S_Type;
# 2251 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile const uint32_t UNUSED;
} NRF_FPU_Type;
# 2266 "../../../../../../modules/nrfx/mdk/nrf52.h"
typedef struct {
  volatile const uint32_t RESERVED[321];
  volatile uint32_t OUT;
  volatile uint32_t OUTSET;
  volatile uint32_t OUTCLR;
  volatile const uint32_t IN;
  volatile uint32_t DIR;
  volatile uint32_t DIRSET;
  volatile uint32_t DIRCLR;
  volatile uint32_t LATCH;


  volatile uint32_t DETECTMODE;

  volatile const uint32_t RESERVED1[118];
  volatile uint32_t PIN_CNF[32];

} NRF_GPIO_Type;
# 161 "../../../../../../modules/nrfx/mdk/nrf.h" 2
# 1 "../../../../../../modules/nrfx/mdk/nrf52_bitfields.h" 1
# 162 "../../../../../../modules/nrfx/mdk/nrf.h" 2
# 1 "../../../../../../modules/nrfx/mdk/nrf51_to_nrf52.h" 1
# 163 "../../../../../../modules/nrfx/mdk/nrf.h" 2
# 1 "../../../../../../modules/nrfx/mdk/nrf52_name_change.h" 1
# 164 "../../../../../../modules/nrfx/mdk/nrf.h" 2
# 195 "../../../../../../modules/nrfx/mdk/nrf.h"
# 1 "../../../../../../modules/nrfx/mdk/compiler_abstraction.h" 1
# 195 "../../../../../../modules/nrfx/mdk/compiler_abstraction.h"
    static inline unsigned int gcc_current_sp(void)
    {
        unsigned int stack_pointer = 0;
        __asm__ __volatile__ ("mov %0, sp" : "=r"(stack_pointer));
        return stack_pointer;
    }
# 196 "../../../../../../modules/nrfx/mdk/nrf.h" 2
# 49 "../../../../../../modules/nrfx/drivers/nrfx_common.h" 2
# 1 "../../../../../../modules/nrfx/mdk/nrf_peripherals.h" 1
# 59 "../../../../../../modules/nrfx/mdk/nrf_peripherals.h"
# 1 "../../../../../../modules/nrfx/mdk/nrf52832_peripherals.h" 1
# 60 "../../../../../../modules/nrfx/mdk/nrf_peripherals.h" 2
# 50 "../../../../../../modules/nrfx/drivers/nrfx_common.h" 2
# 224 "../../../../../../modules/nrfx/drivers/nrfx_common.h"
typedef void (* nrfx_irq_handler_t)(void);


typedef enum
{
    NRFX_DRV_STATE_UNINITIALIZED,
    NRFX_DRV_STATE_INITIALIZED,
    NRFX_DRV_STATE_POWERED_ON,
} nrfx_drv_state_t;
# 247 "../../../../../../modules/nrfx/drivers/nrfx_common.h"
static inline 
# 247 "../../../../../../modules/nrfx/drivers/nrfx_common.h" 3 4
               _Bool 
# 247 "../../../../../../modules/nrfx/drivers/nrfx_common.h"
                    nrfx_is_in_ram(void const * p_object);
# 261 "../../../../../../modules/nrfx/drivers/nrfx_common.h"
static inline 
# 261 "../../../../../../modules/nrfx/drivers/nrfx_common.h" 3 4
               _Bool 
# 261 "../../../../../../modules/nrfx/drivers/nrfx_common.h"
                    nrfx_is_word_aligned(void const * p_object);
# 270 "../../../../../../modules/nrfx/drivers/nrfx_common.h"
static inline IRQn_Type nrfx_get_irq_number(void const * p_reg);
# 286 "../../../../../../modules/nrfx/drivers/nrfx_common.h"
static inline uint32_t nrfx_bitpos_to_event(uint32_t bit);
# 302 "../../../../../../modules/nrfx/drivers/nrfx_common.h"
static inline uint32_t nrfx_event_to_bitpos(uint32_t event);




static inline 
# 307 "../../../../../../modules/nrfx/drivers/nrfx_common.h" 3 4
               _Bool 
# 307 "../../../../../../modules/nrfx/drivers/nrfx_common.h"
                    nrfx_is_in_ram(void const * p_object)
{
    return ((((uint32_t)p_object) & 0xE0000000u) == 0x20000000u);
}

static inline 
# 312 "../../../../../../modules/nrfx/drivers/nrfx_common.h" 3 4
               _Bool 
# 312 "../../../../../../modules/nrfx/drivers/nrfx_common.h"
                    nrfx_is_word_aligned(void const * p_object)
{
    return ((((uint32_t)p_object) & 0x3u) == 0u);
}

static inline IRQn_Type nrfx_get_irq_number(void const * p_reg)
{
    return (IRQn_Type)(uint8_t)((uint32_t)(p_reg) >> 12);
}

static inline uint32_t nrfx_bitpos_to_event(uint32_t bit)
{
    static const uint32_t event_reg_offset = 0x100u;
    return event_reg_offset + (bit * sizeof(uint32_t));
}

static inline uint32_t nrfx_event_to_bitpos(uint32_t event)
{
    static const uint32_t event_reg_offset = 0x100u;
    return (event - event_reg_offset) / sizeof(uint32_t);
}
# 46 "../../../../../../modules/nrfx/nrfx.h" 2
# 1 "../../../../../../integration/nrfx/nrfx_glue.h" 1
# 57 "../../../../../../integration/nrfx/nrfx_glue.h"
# 1 "../../../../../../integration/nrfx/legacy/apply_old_config.h" 1
# 58 "../../../../../../integration/nrfx/nrfx_glue.h" 2

# 1 "../../../../../../modules/nrfx/soc/nrfx_irqs.h" 1
# 53 "../../../../../../modules/nrfx/soc/nrfx_irqs.h"
# 1 "../../../../../../modules/nrfx/soc/nrfx_irqs_nrf52832.h" 1
# 54 "../../../../../../modules/nrfx/soc/nrfx_irqs.h" 2
# 60 "../../../../../../integration/nrfx/nrfx_glue.h" 2



# 1 "../../../../../../components/libraries/util/nrf_assert.h" 1
# 75 "../../../../../../components/libraries/util/nrf_assert.h"
void assert_nrf_callback(uint16_t line_num, const uint8_t *file_name);
# 64 "../../../../../../integration/nrfx/nrfx_glue.h" 2







# 1 "../../../../../../components/libraries/util/app_util.h" 1
# 55 "../../../../../../components/libraries/util/app_util.h"
# 1 "../../../../../../modules/nrfx/mdk/compiler_abstraction.h" 1
# 56 "../../../../../../components/libraries/util/app_util.h" 2
# 1 "../../../../../../components/libraries/util/nordic_common.h" 1
# 57 "../../../../../../components/libraries/util/app_util.h" 2
# 83 "../../../../../../components/libraries/util/app_util.h"
extern uint32_t __StackTop;
extern uint32_t __StackLimit;
# 114 "../../../../../../components/libraries/util/app_util.h"
extern uint32_t * _vectors;
extern uint32_t __FLASH1_segment_used_end__;
# 141 "../../../../../../components/libraries/util/app_util.h"
enum
{
    UNIT_0_625_MS = 625,
    UNIT_1_25_MS = 1250,
    UNIT_10_MS = 10000
};
# 316 "../../../../../../components/libraries/util/app_util.h"
typedef uint8_t uint16_le_t[2];


typedef uint8_t uint32_le_t[4];


typedef struct
{
    uint16_t size;
    uint8_t * p_data;
} uint8_array_t;
# 1023 "../../../../../../components/libraries/util/app_util.h"
static inline uint64_t value_rescale(uint32_t value, uint32_t old_unit_reversal, uint16_t new_unit_reversal)
{
    return (uint64_t)((((uint64_t)value * new_unit_reversal) + ((old_unit_reversal) / 2)) / (old_unit_reversal));
}
# 1035 "../../../../../../components/libraries/util/app_util.h"
static inline uint8_t uint16_encode(uint16_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x00FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0xFF00) >> 8);
    return sizeof(uint16_t);
}
# 1049 "../../../../../../components/libraries/util/app_util.h"
static inline uint8_t uint24_encode(uint32_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x000000FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0x0000FF00) >> 8);
    p_encoded_data[2] = (uint8_t) ((value & 0x00FF0000) >> 16);
    return 3;
}
# 1064 "../../../../../../components/libraries/util/app_util.h"
static inline uint8_t uint32_encode(uint32_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x000000FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0x0000FF00) >> 8);
    p_encoded_data[2] = (uint8_t) ((value & 0x00FF0000) >> 16);
    p_encoded_data[3] = (uint8_t) ((value & 0xFF000000) >> 24);
    return sizeof(uint32_t);
}
# 1080 "../../../../../../components/libraries/util/app_util.h"
static inline uint8_t uint40_encode(uint64_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x00000000FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0x000000FF00) >> 8);
    p_encoded_data[2] = (uint8_t) ((value & 0x0000FF0000) >> 16);
    p_encoded_data[3] = (uint8_t) ((value & 0x00FF000000) >> 24);
    p_encoded_data[4] = (uint8_t) ((value & 0xFF00000000) >> 32);
    return 5;
}
# 1097 "../../../../../../components/libraries/util/app_util.h"
static inline uint8_t uint48_encode(uint64_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x0000000000FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0x00000000FF00) >> 8);
    p_encoded_data[2] = (uint8_t) ((value & 0x000000FF0000) >> 16);
    p_encoded_data[3] = (uint8_t) ((value & 0x0000FF000000) >> 24);
    p_encoded_data[4] = (uint8_t) ((value & 0x00FF00000000) >> 32);
    p_encoded_data[5] = (uint8_t) ((value & 0xFF0000000000) >> 40);
    return 6;
}







static inline uint16_t uint16_decode(const uint8_t * p_encoded_data)
{
        return ( (((uint16_t)((uint8_t *)p_encoded_data)[0])) |
                 (((uint16_t)((uint8_t *)p_encoded_data)[1]) << 8 ));
}







static inline uint16_t uint16_big_decode(const uint8_t * p_encoded_data)
{
        return ( (((uint16_t)((uint8_t *)p_encoded_data)[0]) << 8 ) |
                 (((uint16_t)((uint8_t *)p_encoded_data)[1])) );
}







static inline uint32_t uint24_decode(const uint8_t * p_encoded_data)
{
    return ( (((uint32_t)((uint8_t *)p_encoded_data)[0]) << 0) |
             (((uint32_t)((uint8_t *)p_encoded_data)[1]) << 8) |
             (((uint32_t)((uint8_t *)p_encoded_data)[2]) << 16));
}







static inline uint32_t uint32_decode(const uint8_t * p_encoded_data)
{
    return ( (((uint32_t)((uint8_t *)p_encoded_data)[0]) << 0) |
             (((uint32_t)((uint8_t *)p_encoded_data)[1]) << 8) |
             (((uint32_t)((uint8_t *)p_encoded_data)[2]) << 16) |
             (((uint32_t)((uint8_t *)p_encoded_data)[3]) << 24 ));
}







static inline uint32_t uint32_big_decode(const uint8_t * p_encoded_data)
{
    return ( (((uint32_t)((uint8_t *)p_encoded_data)[0]) << 24) |
             (((uint32_t)((uint8_t *)p_encoded_data)[1]) << 16) |
             (((uint32_t)((uint8_t *)p_encoded_data)[2]) << 8) |
             (((uint32_t)((uint8_t *)p_encoded_data)[3]) << 0) );
}
# 1181 "../../../../../../components/libraries/util/app_util.h"
static inline uint8_t uint16_big_encode(uint16_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) (value >> 8);
    p_encoded_data[1] = (uint8_t) (value & 0xFF);

    return sizeof(uint16_t);
}
# 1199 "../../../../../../components/libraries/util/app_util.h"
static inline uint8_t uint32_big_encode(uint32_t value, uint8_t * p_encoded_data)
{
    *(uint32_t *)p_encoded_data = __REV(value);
    return sizeof(uint32_t);
}







static inline uint64_t uint40_decode(const uint8_t * p_encoded_data)
{
    return ( (((uint64_t)((uint8_t *)p_encoded_data)[0]) << 0) |
             (((uint64_t)((uint8_t *)p_encoded_data)[1]) << 8) |
             (((uint64_t)((uint8_t *)p_encoded_data)[2]) << 16) |
             (((uint64_t)((uint8_t *)p_encoded_data)[3]) << 24) |
             (((uint64_t)((uint8_t *)p_encoded_data)[4]) << 32 ));
}







static inline uint64_t uint48_decode(const uint8_t * p_encoded_data)
{
    return ( (((uint64_t)((uint8_t *)p_encoded_data)[0]) << 0) |
             (((uint64_t)((uint8_t *)p_encoded_data)[1]) << 8) |
             (((uint64_t)((uint8_t *)p_encoded_data)[2]) << 16) |
             (((uint64_t)((uint8_t *)p_encoded_data)[3]) << 24) |
             (((uint64_t)((uint8_t *)p_encoded_data)[4]) << 32) |
             (((uint64_t)((uint8_t *)p_encoded_data)[5]) << 40 ));
}
# 1256 "../../../../../../components/libraries/util/app_util.h"
static inline uint8_t battery_level_in_percent(const uint16_t mvolts)
{
    uint8_t battery_level;

    if (mvolts >= 3000)
    {
        battery_level = 100;
    }
    else if (mvolts > 2900)
    {
        battery_level = 100 - ((3000 - mvolts) * 58) / 100;
    }
    else if (mvolts > 2740)
    {
        battery_level = 42 - ((2900 - mvolts) * 24) / 160;
    }
    else if (mvolts > 2440)
    {
        battery_level = 18 - ((2740 - mvolts) * 12) / 300;
    }
    else if (mvolts > 2100)
    {
        battery_level = 6 - ((2440 - mvolts) * 6) / 340;
    }
    else
    {
        battery_level = 0;
    }

    return battery_level;
}







static inline 
# 1294 "../../../../../../components/libraries/util/app_util.h" 3 4
               _Bool 
# 1294 "../../../../../../components/libraries/util/app_util.h"
                    is_word_aligned(void const* p)
{
    return (((uintptr_t)p & 0x03) == 0);
}
# 1307 "../../../../../../components/libraries/util/app_util.h"
static inline 
# 1307 "../../../../../../components/libraries/util/app_util.h" 3 4
               _Bool 
# 1307 "../../../../../../components/libraries/util/app_util.h"
                    is_address_from_stack(void * ptr)
{
    if (((uint32_t)ptr >= (uint32_t)&__StackLimit) &&
        ((uint32_t)ptr < (uint32_t)&__StackTop) )
    {
        return 
# 1312 "../../../../../../components/libraries/util/app_util.h" 3 4
              1
# 1312 "../../../../../../components/libraries/util/app_util.h"
                  ;
    }
    else
    {
        return 
# 1316 "../../../../../../components/libraries/util/app_util.h" 3 4
              0
# 1316 "../../../../../../components/libraries/util/app_util.h"
                   ;
    }
}
# 72 "../../../../../../integration/nrfx/nrfx_glue.h" 2
# 104 "../../../../../../integration/nrfx/nrfx_glue.h"
static inline void _NRFX_IRQ_PRIORITY_SET(IRQn_Type irq_number,
                                          uint8_t priority)
{
    if (0) { if (((priority) < 8)) { } else { assert_nrf_callback((uint16_t)107, (uint8_t *)"../../../../../../integration/nrfx/nrfx_glue.h"); } };
    __NVIC_SetPriority(irq_number, priority);
}







static inline void _NRFX_IRQ_ENABLE(IRQn_Type irq_number)
{
    __NVIC_EnableIRQ(irq_number);
}
# 131 "../../../../../../integration/nrfx/nrfx_glue.h"
static inline 
# 131 "../../../../../../integration/nrfx/nrfx_glue.h" 3 4
             _Bool 
# 131 "../../../../../../integration/nrfx/nrfx_glue.h"
                  _NRFX_IRQ_IS_ENABLED(IRQn_Type irq_number)
{
    return 0 != (((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[irq_number / 32] & (1UL << (irq_number % 32)));
}







static inline void _NRFX_IRQ_DISABLE(IRQn_Type irq_number)
{
    __NVIC_DisableIRQ(irq_number);
}







static inline void _NRFX_IRQ_PENDING_SET(IRQn_Type irq_number)
{
    __NVIC_SetPendingIRQ(irq_number);
}







static inline void _NRFX_IRQ_PENDING_CLEAR(IRQn_Type irq_number)
{
    __NVIC_ClearPendingIRQ(irq_number);
}
# 176 "../../../../../../integration/nrfx/nrfx_glue.h"
static inline 
# 176 "../../../../../../integration/nrfx/nrfx_glue.h" 3 4
             _Bool 
# 176 "../../../../../../integration/nrfx/nrfx_glue.h"
                  _NRFX_IRQ_IS_PENDING(IRQn_Type irq_number)
{
    return (__NVIC_GetPendingIRQ(irq_number) == 1);
}

# 1 "../../../../../../components/libraries/util/nordic_common.h" 1
# 182 "../../../../../../integration/nrfx/nrfx_glue.h" 2
# 1 "../../../../../../components/libraries/util/app_util_platform.h" 1
# 59 "../../../../../../components/libraries/util/app_util_platform.h"
# 1 "../../../../../../components/libraries/util/nrf_assert.h" 1
# 60 "../../../../../../components/libraries/util/app_util_platform.h" 2
# 1 "../../../../../../components/libraries/util/app_error.h" 1
# 54 "../../../../../../components/libraries/util/app_error.h"
# 1 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/stdio.h" 1 3 4
# 77 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/stdio.h" 3 4

# 77 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/stdio.h" 3 4
typedef long fpos_t;




typedef struct __SEGGER_RTL_FILE_impl FILE;




typedef struct __SEGGER_RTL_POSIX_locale_s *locale_t;
# 97 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/stdio.h" 3 4
extern FILE *stdin;
extern FILE *stdout;
extern FILE *stderr;
# 108 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/stdio.h" 3 4
int putchar (int __c);
int getchar (void);
int puts (const char *__s);
char * gets (char *__s);
int sprintf (char *__s, const char *__format, ...);
int sprintf_l (char *__s, locale_t __loc, const char *__format, ...);
int snprintf (char *__s, size_t __n, const char *__format, ...);
int snprintf_l (char *__s, size_t __n, locale_t __loc, const char *__format, ...);
int vsnprintf (char *__s, size_t __n, const char *__format, __builtin_va_list __arg);
int vsnprintf_l (char *__s, size_t __n, locale_t __loc, const char *__format, __builtin_va_list __arg);
int printf (const char *__format, ...);
int printf_l (locale_t __loc, const char *__format, ...);
int vprintf (const char *__format, __builtin_va_list __arg);
int vprintf_l (locale_t __loc, const char *__format, __builtin_va_list __arg);
int vsprintf (char *__s, const char *__format, __builtin_va_list __arg);
int vsprintf_l (char *__s, locale_t __loc, const char *__format, __builtin_va_list __arg);
int vasprintf (char **__strp, const char *__format, __builtin_va_list __ap);
int vasprintf_l (char **__strp, locale_t __loc, const char *__format, __builtin_va_list __ap);
int asprintf (char **__strp, const char *__format, ...);
int asprintf_l (char **__strp, locale_t __loc, const char *__format, ...);

int scanf (const char *__format, ...);
int scanf_l (locale_t __loc, const char *__format, ...);
int sscanf (const char *__s, const char *__format, ...);
int sscanf_l (const char *__s, locale_t __loc, const char *__format, ...);
int vscanf (const char *__format, __builtin_va_list __arg);
int vscanf_l (locale_t __loc, const char *__format, __builtin_va_list __arg);
int vsscanf (const char *__s, const char *__format, __builtin_va_list __arg);
int vsscanf_l (const char *__s, locale_t __loc, const char *__format, __builtin_va_list __arg);

void clearerr (FILE *);
int fclose (FILE *);
int feof (FILE *);
int ferror (FILE *);
int fflush (FILE *);
int fgetc (FILE *);
int fgetpos (FILE *, fpos_t *);
char * fgets (char *, int, FILE *);
int fileno (FILE *);
FILE * fopen (const char *, const char *);
int fprintf (FILE *, const char *, ...);
int fprintf_l (FILE *, locale_t, const char *, ...);
int vfprintf (FILE *__stream, const char *__format, __builtin_va_list __arg);
int vfprintf_l (FILE *__stream, locale_t loc, const char *__format, __builtin_va_list __arg);
int fputc (int, FILE *);
int fputs (const char *, FILE *);
size_t fread (void *, size_t, size_t, FILE *);
FILE * freopen (const char *, const char *, FILE *);
int fscanf (FILE *, const char *, ...);
int fscanf_l (FILE *, locale_t, const char *, ...);
int vfscanf (FILE *, const char *, __builtin_va_list);
int vfscanf_l (FILE *, locale_t, const char *, __builtin_va_list);
int fseek (FILE *, long, int);
int fsetpos (FILE *, const fpos_t *);
long ftell (FILE *);
size_t fwrite (const void *, size_t, size_t, FILE *);
int getc (FILE *);
void perror (const char *);
int putc (int, FILE *);
int remove (const char *);
int rename (const char *, const char *);
void rewind (FILE *);
void setbuf (FILE *, char *);
int setvbuf (FILE *, char *, int, size_t);
FILE * tmpfile (void);
char * tmpnam (char *);
int ungetc (int, FILE *);





int dprintf (int, const char *, ...);
int vdprintf (int, const char *, __builtin_va_list);
FILE * fdopen (int, const char *);
# 55 "../../../../../../components/libraries/util/app_error.h" 2


# 1 "../../../../../../components/libraries/util/sdk_errors.h" 1
# 58 "../../../../../../components/libraries/util/app_error.h" 2

# 1 "../../../../../../components/libraries/util/app_error_weak.h" 1
# 77 "../../../../../../components/libraries/util/app_error_weak.h"

# 77 "../../../../../../components/libraries/util/app_error_weak.h"
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info);
# 60 "../../../../../../components/libraries/util/app_error.h" 2
# 80 "../../../../../../components/libraries/util/app_error.h"
typedef struct
{
    uint32_t line_num;
    uint8_t const * p_file_name;
    uint32_t err_code;
} error_info_t;



typedef struct
{
    uint32_t line_num;
    uint8_t const * p_file_name;
} assert_info_t;
# 111 "../../../../../../components/libraries/util/app_error.h"
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name);





void app_error_handler_bare(ret_code_t error_code);
# 127 "../../../../../../components/libraries/util/app_error.h"
void app_error_save_and_stop(uint32_t id, uint32_t pc, uint32_t info);
# 137 "../../../../../../components/libraries/util/app_error.h"
void app_error_log_handle(uint32_t id, uint32_t pc, uint32_t info);
# 61 "../../../../../../components/libraries/util/app_util_platform.h" 2
# 92 "../../../../../../components/libraries/util/app_util_platform.h"
typedef enum
{

    APP_IRQ_PRIORITY_HIGHEST = 0,



    APP_IRQ_PRIORITY_HIGH = 2,

    APP_IRQ_PRIORITY_MID = 4,



    APP_IRQ_PRIORITY_LOW_MID = 5,
    APP_IRQ_PRIORITY_LOW = 6,
    APP_IRQ_PRIORITY_LOWEST = 7,
    APP_IRQ_PRIORITY_THREAD = 15
} app_irq_priority_t;




typedef enum
{
    APP_LEVEL_UNPRIVILEGED,
    APP_LEVEL_PRIVILEGED
} app_level_t;
# 174 "../../../../../../components/libraries/util/app_util_platform.h"
void app_util_critical_region_enter (uint8_t *p_nested);
void app_util_critical_region_exit (uint8_t nested);
# 260 "../../../../../../components/libraries/util/app_util_platform.h"
uint8_t current_int_priority_get(void);
# 269 "../../../../../../components/libraries/util/app_util_platform.h"
uint8_t privilege_level_get(void);
# 183 "../../../../../../integration/nrfx/nrfx_glue.h" 2
# 203 "../../../../../../integration/nrfx/nrfx_glue.h"
# 1 "../../../../../../modules/nrfx/soc/nrfx_coredep.h" 1
# 92 "../../../../../../modules/nrfx/soc/nrfx_coredep.h"
static inline void nrfx_coredep_delay_us(uint32_t time_us);
# 136 "../../../../../../modules/nrfx/soc/nrfx_coredep.h"
static inline void nrfx_coredep_delay_us(uint32_t time_us)
{
    if (time_us == 0)
    {
        return;
    }
# 161 "../../../../../../modules/nrfx/soc/nrfx_coredep.h"
    __attribute__((aligned(16)))
    static const uint16_t delay_machine_code[] = {
        0x3800 + 3,
        0xd8fd,
        0x4770
    };

    typedef void (* delay_func_t)(uint32_t);
    const delay_func_t delay_cycles =

        (delay_func_t)((((uint32_t)delay_machine_code) | 1));
    uint32_t cycles = time_us * 64;
    delay_cycles(cycles);
}
# 204 "../../../../../../integration/nrfx/nrfx_glue.h" 2





# 1 "../../../../../../modules/nrfx/soc/nrfx_atomic.h" 1
# 44 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"
# 1 "../../../../../../modules/nrfx/nrfx.h" 1
# 45 "../../../../../../modules/nrfx/soc/nrfx_atomic.h" 2
# 63 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"
typedef volatile uint32_t nrfx_atomic_u32_t;


typedef volatile uint32_t nrfx_atomic_flag_t;
# 76 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"
uint32_t nrfx_atomic_u32_fetch_store(nrfx_atomic_u32_t * p_data, uint32_t value);
# 86 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"
uint32_t nrfx_atomic_u32_store(nrfx_atomic_u32_t * p_data, uint32_t value);
# 97 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"
uint32_t nrfx_atomic_u32_fetch_or(nrfx_atomic_u32_t * p_data, uint32_t value);
# 108 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"
uint32_t nrfx_atomic_u32_or(nrfx_atomic_u32_t * p_data, uint32_t value);
# 119 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"
uint32_t nrfx_atomic_u32_fetch_and(nrfx_atomic_u32_t * p_data, uint32_t value);
# 130 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"
uint32_t nrfx_atomic_u32_and(nrfx_atomic_u32_t * p_data, uint32_t value);
# 141 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"
uint32_t nrfx_atomic_u32_fetch_xor(nrfx_atomic_u32_t * p_data, uint32_t value);
# 152 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"
uint32_t nrfx_atomic_u32_xor(nrfx_atomic_u32_t * p_data, uint32_t value);
# 163 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"
uint32_t nrfx_atomic_u32_fetch_add(nrfx_atomic_u32_t * p_data, uint32_t value);
# 174 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"
uint32_t nrfx_atomic_u32_add(nrfx_atomic_u32_t * p_data, uint32_t value);
# 185 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"
uint32_t nrfx_atomic_u32_fetch_sub(nrfx_atomic_u32_t * p_data, uint32_t value);
# 196 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"
uint32_t nrfx_atomic_u32_sub(nrfx_atomic_u32_t * p_data, uint32_t value);
# 212 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"

# 212 "../../../../../../modules/nrfx/soc/nrfx_atomic.h" 3 4
_Bool 
# 212 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"
    nrfx_atomic_u32_cmp_exch(nrfx_atomic_u32_t * p_data,
                              uint32_t * p_expected,
                              uint32_t desired);
# 225 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"
uint32_t nrfx_atomic_u32_fetch_sub_hs(nrfx_atomic_u32_t * p_data, uint32_t value);
# 236 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"
uint32_t nrfx_atomic_u32_sub_hs(nrfx_atomic_u32_t * p_data, uint32_t value);
# 246 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"
uint32_t nrfx_atomic_flag_set_fetch(nrfx_atomic_flag_t * p_data);
# 256 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"
uint32_t nrfx_atomic_flag_set(nrfx_atomic_flag_t * p_data);
# 266 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"
uint32_t nrfx_atomic_flag_clear_fetch(nrfx_atomic_flag_t * p_data);
# 276 "../../../../../../modules/nrfx/soc/nrfx_atomic.h"
uint32_t nrfx_atomic_flag_clear(nrfx_atomic_flag_t * p_data);
# 210 "../../../../../../integration/nrfx/nrfx_glue.h" 2
# 288 "../../../../../../integration/nrfx/nrfx_glue.h"
typedef ret_code_t nrfx_err_t;
# 311 "../../../../../../integration/nrfx/nrfx_glue.h"
# 1 "../../../../../../components/libraries/util/sdk_resources.h" 1
# 312 "../../../../../../integration/nrfx/nrfx_glue.h" 2
# 47 "../../../../../../modules/nrfx/nrfx.h" 2
# 1 "../../../../../../modules/nrfx/drivers/nrfx_errors.h" 1
# 48 "../../../../../../modules/nrfx/nrfx.h" 2
# 45 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 2

# 1 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h" 1
# 45 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h"
# 1 "../../../../../../modules/nrfx/drivers/include/nrfx_twi_twim.h" 1
# 50 "../../../../../../modules/nrfx/drivers/include/nrfx_twi_twim.h"
nrfx_err_t nrfx_twi_twim_bus_recover(uint32_t scl_pin, uint32_t sda_pin);
# 46 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h" 2
# 1 "../../../../../../modules/nrfx/hal/nrf_twim.h" 1
# 58 "../../../../../../modules/nrfx/hal/nrf_twim.h"
typedef enum
{
    NRF_TWIM_TASK_STARTRX = __builtin_offsetof (NRF_TWIM_Type, TASKS_STARTRX),
    NRF_TWIM_TASK_STARTTX = __builtin_offsetof (NRF_TWIM_Type, TASKS_STARTTX),
    NRF_TWIM_TASK_STOP = __builtin_offsetof (NRF_TWIM_Type, TASKS_STOP),
    NRF_TWIM_TASK_SUSPEND = __builtin_offsetof (NRF_TWIM_Type, TASKS_SUSPEND),
    NRF_TWIM_TASK_RESUME = __builtin_offsetof (NRF_TWIM_Type, TASKS_RESUME)
} nrf_twim_task_t;


typedef enum
{
    NRF_TWIM_EVENT_STOPPED = __builtin_offsetof (NRF_TWIM_Type, EVENTS_STOPPED),
    NRF_TWIM_EVENT_ERROR = __builtin_offsetof (NRF_TWIM_Type, EVENTS_ERROR),
    NRF_TWIM_EVENT_SUSPENDED = 0x148,
    NRF_TWIM_EVENT_RXSTARTED = __builtin_offsetof (NRF_TWIM_Type, EVENTS_RXSTARTED),
    NRF_TWIM_EVENT_TXSTARTED = __builtin_offsetof (NRF_TWIM_Type, EVENTS_TXSTARTED),
    NRF_TWIM_EVENT_LASTRX = __builtin_offsetof (NRF_TWIM_Type, EVENTS_LASTRX),
    NRF_TWIM_EVENT_LASTTX = __builtin_offsetof (NRF_TWIM_Type, EVENTS_LASTTX)
} nrf_twim_event_t;


typedef enum
{
    NRF_TWIM_SHORT_LASTTX_STARTRX_MASK = (0x1UL << (7UL)),
    NRF_TWIM_SHORT_LASTTX_SUSPEND_MASK = (0x1UL << (8UL)),
    NRF_TWIM_SHORT_LASTTX_STOP_MASK = (0x1UL << (9UL)),
    NRF_TWIM_SHORT_LASTRX_STARTTX_MASK = (0x1UL << (10UL)),
    NRF_TWIM_SHORT_LASTRX_STOP_MASK = (0x1UL << (12UL)),
    NRF_TWIM_ALL_SHORTS_MASK = (0x1UL << (7UL)) |
                                         (0x1UL << (8UL)) |
                                         (0x1UL << (9UL)) |
                                         (0x1UL << (10UL)) |
                                         (0x1UL << (12UL))
} nrf_twim_short_mask_t;


typedef enum
{
    NRF_TWIM_INT_STOPPED_MASK = (0x1UL << (1UL)),
    NRF_TWIM_INT_ERROR_MASK = (0x1UL << (9UL)),
    NRF_TWIM_INT_SUSPENDED_MASK = (0x1UL << (18UL)),
    NRF_TWIM_INT_RXSTARTED_MASK = (0x1UL << (19UL)),
    NRF_TWIM_INT_TXSTARTED_MASK = (0x1UL << (20UL)),
    NRF_TWIM_INT_LASTRX_MASK = (0x1UL << (23UL)),
    NRF_TWIM_INT_LASTTX_MASK = (0x1UL << (24UL)),
    NRF_TWIM_ALL_INTS_MASK = (0x1UL << (1UL)) |
                                  (0x1UL << (9UL)) |
                                  (0x1UL << (18UL)) |
                                  (0x1UL << (19UL)) |
                                  (0x1UL << (20UL)) |
                                  (0x1UL << (23UL)) |
                                  (0x1UL << (24UL))
} nrf_twim_int_mask_t;


typedef enum
{
    NRF_TWIM_FREQ_100K = (0x01980000UL),
    NRF_TWIM_FREQ_250K = (0x04000000UL),
    NRF_TWIM_FREQ_400K = (0x06400000UL)
} nrf_twim_frequency_t;


typedef enum
{
    NRF_TWIM_ERROR_ADDRESS_NACK = (0x1UL << (1UL)),
    NRF_TWIM_ERROR_DATA_NACK = (0x1UL << (2UL)),
    NRF_TWIM_ERROR_OVERRUN = (0x1UL << (0UL))


} nrf_twim_error_t;
# 138 "../../../../../../modules/nrfx/hal/nrf_twim.h"
static inline void nrf_twim_task_trigger(NRF_TWIM_Type * p_reg,
                                           nrf_twim_task_t task);
# 149 "../../../../../../modules/nrfx/hal/nrf_twim.h"
static inline uint32_t * nrf_twim_task_address_get(NRF_TWIM_Type * p_reg,
                                                     nrf_twim_task_t task);







static inline void nrf_twim_event_clear(NRF_TWIM_Type * p_reg,
                                          nrf_twim_event_t event);
# 170 "../../../../../../modules/nrfx/hal/nrf_twim.h"
static inline 
# 170 "../../../../../../modules/nrfx/hal/nrf_twim.h" 3 4
               _Bool 
# 170 "../../../../../../modules/nrfx/hal/nrf_twim.h"
                    nrf_twim_event_check(NRF_TWIM_Type * p_reg,
                                          nrf_twim_event_t event);
# 181 "../../../../../../modules/nrfx/hal/nrf_twim.h"
static inline uint32_t * nrf_twim_event_address_get(NRF_TWIM_Type * p_reg,
                                                      nrf_twim_event_t event);







static inline void nrf_twim_shorts_enable(NRF_TWIM_Type * p_reg,
                                            uint32_t mask);







static inline void nrf_twim_shorts_disable(NRF_TWIM_Type * p_reg,
                                             uint32_t mask);







static inline void nrf_twim_int_enable(NRF_TWIM_Type * p_reg,
                                         uint32_t mask);







static inline void nrf_twim_int_disable(NRF_TWIM_Type * p_reg,
                                          uint32_t mask);
# 229 "../../../../../../modules/nrfx/hal/nrf_twim.h"
static inline 
# 229 "../../../../../../modules/nrfx/hal/nrf_twim.h" 3 4
               _Bool 
# 229 "../../../../../../modules/nrfx/hal/nrf_twim.h"
                    nrf_twim_int_enable_check(NRF_TWIM_Type * p_reg,
                                               nrf_twim_int_mask_t twim_int);
# 283 "../../../../../../modules/nrfx/hal/nrf_twim.h"
static inline void nrf_twim_enable(NRF_TWIM_Type * p_reg);






static inline void nrf_twim_disable(NRF_TWIM_Type * p_reg);
# 299 "../../../../../../modules/nrfx/hal/nrf_twim.h"
static inline void nrf_twim_pins_set(NRF_TWIM_Type * p_reg,
                                       uint32_t scl_pin,
                                       uint32_t sda_pin);







static inline void nrf_twim_frequency_set(NRF_TWIM_Type * p_reg,
                                            nrf_twim_frequency_t frequency);
# 321 "../../../../../../modules/nrfx/hal/nrf_twim.h"
static inline uint32_t nrf_twim_errorsrc_get_and_clear(NRF_TWIM_Type * p_reg);







static inline void nrf_twim_address_set(NRF_TWIM_Type * p_reg,
                                          uint8_t address);
# 339 "../../../../../../modules/nrfx/hal/nrf_twim.h"
static inline void nrf_twim_tx_buffer_set(NRF_TWIM_Type * p_reg,
                                            uint8_t const * p_buffer,
                                            size_t length);
# 350 "../../../../../../modules/nrfx/hal/nrf_twim.h"
static inline void nrf_twim_rx_buffer_set(NRF_TWIM_Type * p_reg,
                                            uint8_t * p_buffer,
                                            size_t length);







static inline void nrf_twim_shorts_set(NRF_TWIM_Type * p_reg,
                                         uint32_t mask);
# 370 "../../../../../../modules/nrfx/hal/nrf_twim.h"
static inline uint32_t nrf_twim_shorts_get(NRF_TWIM_Type const * p_reg);
# 379 "../../../../../../modules/nrfx/hal/nrf_twim.h"
static inline size_t nrf_twim_txd_amount_get(NRF_TWIM_Type * p_reg);
# 388 "../../../../../../modules/nrfx/hal/nrf_twim.h"
static inline size_t nrf_twim_rxd_amount_get(NRF_TWIM_Type * p_reg);






static inline void nrf_twim_tx_list_enable(NRF_TWIM_Type * p_reg);






static inline void nrf_twim_tx_list_disable(NRF_TWIM_Type * p_reg);






static inline void nrf_twim_rx_list_enable(NRF_TWIM_Type * p_reg);






static inline void nrf_twim_rx_list_disable(NRF_TWIM_Type * p_reg);




static inline void nrf_twim_task_trigger(NRF_TWIM_Type * p_reg,
                                           nrf_twim_task_t task)
{
    *((volatile uint32_t *)((uint8_t *)p_reg + (uint32_t)task)) = 0x1UL;
}

static inline uint32_t * nrf_twim_task_address_get(NRF_TWIM_Type * p_reg,
                                                     nrf_twim_task_t task)
{
    return (uint32_t *)((uint8_t *)p_reg + (uint32_t)task);
}

static inline void nrf_twim_event_clear(NRF_TWIM_Type * p_reg,
                                          nrf_twim_event_t event)
{
    *((volatile uint32_t *)((uint8_t *)p_reg + (uint32_t)event)) = 0x0UL;

    volatile uint32_t dummy = *((volatile uint32_t *)((uint8_t *)p_reg + (uint32_t)event));
    (void)dummy;

}

static inline 
# 443 "../../../../../../modules/nrfx/hal/nrf_twim.h" 3 4
               _Bool 
# 443 "../../../../../../modules/nrfx/hal/nrf_twim.h"
                    nrf_twim_event_check(NRF_TWIM_Type * p_reg,
                                          nrf_twim_event_t event)
{
    return (
# 446 "../../../../../../modules/nrfx/hal/nrf_twim.h" 3 4
           _Bool
# 446 "../../../../../../modules/nrfx/hal/nrf_twim.h"
               )*(volatile uint32_t *)((uint8_t *)p_reg + (uint32_t)event);
}

static inline uint32_t * nrf_twim_event_address_get(NRF_TWIM_Type * p_reg,
                                                      nrf_twim_event_t event)
{
    return (uint32_t *)((uint8_t *)p_reg + (uint32_t)event);
}

static inline void nrf_twim_shorts_enable(NRF_TWIM_Type * p_reg,
                                            uint32_t mask)
{
    p_reg->SHORTS |= mask;
}

static inline void nrf_twim_shorts_disable(NRF_TWIM_Type * p_reg,
                                             uint32_t mask)
{
    p_reg->SHORTS &= ~(mask);
}

static inline void nrf_twim_int_enable(NRF_TWIM_Type * p_reg,
                                         uint32_t mask)
{
    p_reg->INTENSET = mask;
}

static inline void nrf_twim_int_disable(NRF_TWIM_Type * p_reg,
                                          uint32_t mask)
{
    p_reg->INTENCLR = mask;
}

static inline 
# 479 "../../../../../../modules/nrfx/hal/nrf_twim.h" 3 4
               _Bool 
# 479 "../../../../../../modules/nrfx/hal/nrf_twim.h"
                    nrf_twim_int_enable_check(NRF_TWIM_Type * p_reg,
                                               nrf_twim_int_mask_t twim_int)
{
    return (
# 482 "../../../../../../modules/nrfx/hal/nrf_twim.h" 3 4
           _Bool
# 482 "../../../../../../modules/nrfx/hal/nrf_twim.h"
               )(p_reg->INTENSET & twim_int);
}
# 515 "../../../../../../modules/nrfx/hal/nrf_twim.h"
static inline void nrf_twim_enable(NRF_TWIM_Type * p_reg)
{
    p_reg->ENABLE = ((6UL) << (0UL));
}

static inline void nrf_twim_disable(NRF_TWIM_Type * p_reg)
{
    p_reg->ENABLE = ((0UL) << (0UL));
}

static inline void nrf_twim_pins_set(NRF_TWIM_Type * p_reg,
                                       uint32_t scl_pin,
                                       uint32_t sda_pin)
{
    p_reg->PSEL.SCL = scl_pin;
    p_reg->PSEL.SDA = sda_pin;
}

static inline void nrf_twim_frequency_set(NRF_TWIM_Type * p_reg,
                                            nrf_twim_frequency_t frequency)
{
    p_reg->FREQUENCY = frequency;
}

static inline uint32_t nrf_twim_errorsrc_get_and_clear(NRF_TWIM_Type * p_reg)
{
    uint32_t error_source = p_reg->ERRORSRC;


    p_reg->ERRORSRC = error_source;

    return error_source;
}

static inline void nrf_twim_address_set(NRF_TWIM_Type * p_reg,
                                          uint8_t address)
{
    p_reg->ADDRESS = address;
}

static inline void nrf_twim_tx_buffer_set(NRF_TWIM_Type * p_reg,
                                            uint8_t const * p_buffer,
                                            size_t length)
{
    p_reg->TXD.PTR = (uint32_t)p_buffer;
    p_reg->TXD.MAXCNT = length;
}

static inline void nrf_twim_rx_buffer_set(NRF_TWIM_Type * p_reg,
                                            uint8_t * p_buffer,
                                            size_t length)
{
    p_reg->RXD.PTR = (uint32_t)p_buffer;
    p_reg->RXD.MAXCNT = length;
}

static inline void nrf_twim_shorts_set(NRF_TWIM_Type * p_reg,
                                         uint32_t mask)
{
    p_reg->SHORTS = mask;
}

static inline size_t nrf_twim_txd_amount_get(NRF_TWIM_Type * p_reg)
{
    return p_reg->TXD.AMOUNT;
}

static inline uint32_t nrf_twim_shorts_get(NRF_TWIM_Type const * p_reg)
{
    return p_reg->SHORTS;
}

static inline size_t nrf_twim_rxd_amount_get(NRF_TWIM_Type * p_reg)
{
    return p_reg->RXD.AMOUNT;
}

static inline void nrf_twim_tx_list_enable(NRF_TWIM_Type * p_reg)
{
    p_reg->TXD.LIST = 1;
}

static inline void nrf_twim_tx_list_disable(NRF_TWIM_Type * p_reg)
{
    p_reg->TXD.LIST = 0;
}

static inline void nrf_twim_rx_list_enable(NRF_TWIM_Type * p_reg)
{
    p_reg->RXD.LIST = 1;
}

static inline void nrf_twim_rx_list_disable(NRF_TWIM_Type * p_reg)
{
    p_reg->RXD.LIST = 0;
}
# 47 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h" 2
# 60 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h"
typedef struct
{
    NRF_TWIM_Type * p_twim;
    uint8_t drv_inst_idx;
} nrfx_twim_t;
# 74 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h"
enum {
# 87 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h"
    NRFX_TWIM_ENABLED_COUNT
};



typedef struct
{
    uint32_t scl;
    uint32_t sda;
    nrf_twim_frequency_t frequency;
    uint8_t interrupt_priority;
    
# 98 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h" 3 4
   _Bool 
# 98 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h"
                        hold_bus_uninit;
} nrfx_twim_config_t;
# 127 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h"
typedef enum
{
    NRFX_TWIM_EVT_DONE,
    NRFX_TWIM_EVT_ADDRESS_NACK,
    NRFX_TWIM_EVT_DATA_NACK,
    NRFX_TWIM_EVT_OVERRUN,
    NRFX_TWIM_EVT_BUS_ERROR
} nrfx_twim_evt_type_t;


typedef enum
{
    NRFX_TWIM_XFER_TX,
    NRFX_TWIM_XFER_RX,
    NRFX_TWIM_XFER_TXRX,
    NRFX_TWIM_XFER_TXTX
} nrfx_twim_xfer_type_t;


typedef struct
{
    nrfx_twim_xfer_type_t type;
    uint8_t address;
    size_t primary_length;
    size_t secondary_length;
    uint8_t * p_primary_buf;
    uint8_t * p_secondary_buf;
} nrfx_twim_xfer_desc_t;
# 202 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h"
typedef struct
{
    nrfx_twim_evt_type_t type;
    nrfx_twim_xfer_desc_t xfer_desc;
} nrfx_twim_evt_t;


typedef void (* nrfx_twim_evt_handler_t)(nrfx_twim_evt_t const * p_event,
                                         void * p_context);
# 227 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h"
nrfx_err_t nrfx_twim_init(nrfx_twim_t const * p_instance,
                          nrfx_twim_config_t const * p_config,
                          nrfx_twim_evt_handler_t event_handler,
                          void * p_context);






void nrfx_twim_uninit(nrfx_twim_t const * p_instance);






void nrfx_twim_enable(nrfx_twim_t const * p_instance);






void nrfx_twim_disable(nrfx_twim_t const * p_instance);
# 283 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h"
nrfx_err_t nrfx_twim_tx(nrfx_twim_t const * p_instance,
                        uint8_t address,
                        uint8_t const * p_data,
                        size_t length,
                        
# 287 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h" 3 4
                       _Bool 
# 287 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h"
                                           no_stop);
# 312 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h"
nrfx_err_t nrfx_twim_rx(nrfx_twim_t const * p_instance,
                        uint8_t address,
                        uint8_t * p_data,
                        size_t length);
# 372 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h"
nrfx_err_t nrfx_twim_xfer(nrfx_twim_t const * p_instance,
                          nrfx_twim_xfer_desc_t const * p_xfer_desc,
                          uint32_t flags);
# 384 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h"

# 384 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h" 3 4
_Bool 
# 384 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h"
    nrfx_twim_is_busy(nrfx_twim_t const * p_instance);
# 398 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h"
uint32_t nrfx_twim_start_task_get(nrfx_twim_t const * p_instance, nrfx_twim_xfer_type_t xfer_type);
# 410 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h"
uint32_t nrfx_twim_stopped_event_get(nrfx_twim_t const * p_instance);
# 427 "../../../../../../modules/nrfx/drivers/include/nrfx_twim.h"
static inline nrfx_err_t nrfx_twim_bus_recover(uint32_t scl_pin, uint32_t sda_pin);


static inline nrfx_err_t nrfx_twim_bus_recover(uint32_t scl_pin, uint32_t sda_pin)
{
    return nrfx_twi_twim_bus_recover(scl_pin, sda_pin);
}




void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void);
void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler(void);
void nrfx_twim_2_irq_handler(void);
void nrfx_twim_3_irq_handler(void);
# 47 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 2
# 63 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
# 1 "../../../../../../modules/nrfx/drivers/include/nrfx_twi.h" 1
# 46 "../../../../../../modules/nrfx/drivers/include/nrfx_twi.h"
# 1 "../../../../../../modules/nrfx/hal/nrf_twi.h" 1
# 58 "../../../../../../modules/nrfx/hal/nrf_twi.h"
typedef enum
{
    NRF_TWI_TASK_STARTRX = __builtin_offsetof (NRF_TWI_Type, TASKS_STARTRX),
    NRF_TWI_TASK_STARTTX = __builtin_offsetof (NRF_TWI_Type, TASKS_STARTTX),
    NRF_TWI_TASK_STOP = __builtin_offsetof (NRF_TWI_Type, TASKS_STOP),
    NRF_TWI_TASK_SUSPEND = __builtin_offsetof (NRF_TWI_Type, TASKS_SUSPEND),
    NRF_TWI_TASK_RESUME = __builtin_offsetof (NRF_TWI_Type, TASKS_RESUME)
} nrf_twi_task_t;


typedef enum
{
    NRF_TWI_EVENT_STOPPED = __builtin_offsetof (NRF_TWI_Type, EVENTS_STOPPED),
    NRF_TWI_EVENT_RXDREADY = __builtin_offsetof (NRF_TWI_Type, EVENTS_RXDREADY),
    NRF_TWI_EVENT_TXDSENT = __builtin_offsetof (NRF_TWI_Type, EVENTS_TXDSENT),
    NRF_TWI_EVENT_ERROR = __builtin_offsetof (NRF_TWI_Type, EVENTS_ERROR),
    NRF_TWI_EVENT_BB = __builtin_offsetof (NRF_TWI_Type, EVENTS_BB),
    NRF_TWI_EVENT_SUSPENDED = __builtin_offsetof (NRF_TWI_Type, EVENTS_SUSPENDED)
} nrf_twi_event_t;


typedef enum
{
    NRF_TWI_SHORT_BB_SUSPEND_MASK = (0x1UL << (0UL)),
    NRF_TWI_SHORT_BB_STOP_MASK = (0x1UL << (1UL)),
    NRF_TWI_ALL_SHORTS_MASK = (0x1UL << (0UL)) |
                                    (0x1UL << (1UL))
} nrf_twi_short_mask_t;


typedef enum
{
    NRF_TWI_INT_STOPPED_MASK = (0x1UL << (1UL)),
    NRF_TWI_INT_RXDREADY_MASK = (0x1UL << (2UL)),
    NRF_TWI_INT_TXDSENT_MASK = (0x1UL << (7UL)),
    NRF_TWI_INT_ERROR_MASK = (0x1UL << (9UL)),
    NRF_TWI_INT_BB_MASK = (0x1UL << (14UL)),
    NRF_TWI_INT_SUSPENDED_MASK = (0x1UL << (18UL)),
    NRF_TWI_ALL_INTS_MASK = (0x1UL << (1UL)) |
                                  (0x1UL << (2UL)) |
                                  (0x1UL << (7UL)) |
                                  (0x1UL << (9UL)) |
                                  (0x1UL << (14UL)) |
                                  (0x1UL << (18UL))
} nrf_twi_int_mask_t;


typedef enum
{
    NRF_TWI_ERROR_ADDRESS_NACK = (0x1UL << (1UL)),
    NRF_TWI_ERROR_DATA_NACK = (0x1UL << (2UL)),
    NRF_TWI_ERROR_OVERRUN = (0x1UL << (0UL))


} nrf_twi_error_t;


typedef enum
{
    NRF_TWI_FREQ_100K = (0x01980000UL),
    NRF_TWI_FREQ_250K = (0x04000000UL),
    NRF_TWI_FREQ_400K = (0x06680000UL)
} nrf_twi_frequency_t;
# 129 "../../../../../../modules/nrfx/hal/nrf_twi.h"
static inline void nrf_twi_task_trigger(NRF_TWI_Type * p_reg,
                                          nrf_twi_task_t task);
# 140 "../../../../../../modules/nrfx/hal/nrf_twi.h"
static inline uint32_t * nrf_twi_task_address_get(NRF_TWI_Type * p_reg,
                                                    nrf_twi_task_t task);







static inline void nrf_twi_event_clear(NRF_TWI_Type * p_reg,
                                         nrf_twi_event_t event);
# 161 "../../../../../../modules/nrfx/hal/nrf_twi.h"
static inline 
# 161 "../../../../../../modules/nrfx/hal/nrf_twi.h" 3 4
               _Bool 
# 161 "../../../../../../modules/nrfx/hal/nrf_twi.h"
                    nrf_twi_event_check(NRF_TWI_Type * p_reg,
                                         nrf_twi_event_t event);
# 172 "../../../../../../modules/nrfx/hal/nrf_twi.h"
static inline uint32_t * nrf_twi_event_address_get(NRF_TWI_Type * p_reg,
                                                     nrf_twi_event_t event);







static inline void nrf_twi_shorts_enable(NRF_TWI_Type * p_reg,
                                           uint32_t mask);







static inline void nrf_twi_shorts_disable(NRF_TWI_Type * p_reg,
                                            uint32_t mask);







static inline void nrf_twi_int_enable(NRF_TWI_Type * p_reg,
                                        uint32_t mask);







static inline void nrf_twi_int_disable(NRF_TWI_Type * p_reg,
                                         uint32_t mask);
# 220 "../../../../../../modules/nrfx/hal/nrf_twi.h"
static inline 
# 220 "../../../../../../modules/nrfx/hal/nrf_twi.h" 3 4
               _Bool 
# 220 "../../../../../../modules/nrfx/hal/nrf_twi.h"
                    nrf_twi_int_enable_check(NRF_TWI_Type * p_reg,
                                              nrf_twi_int_mask_t twi_int);






static inline void nrf_twi_enable(NRF_TWI_Type * p_reg);






static inline void nrf_twi_disable(NRF_TWI_Type * p_reg);
# 244 "../../../../../../modules/nrfx/hal/nrf_twi.h"
static inline void nrf_twi_pins_set(NRF_TWI_Type * p_reg,
                                      uint32_t scl_pin,
                                      uint32_t sda_pin);
# 255 "../../../../../../modules/nrfx/hal/nrf_twi.h"
static inline uint32_t nrf_twi_scl_pin_get(NRF_TWI_Type * p_reg);
# 264 "../../../../../../modules/nrfx/hal/nrf_twi.h"
static inline uint32_t nrf_twi_sda_pin_get(NRF_TWI_Type * p_reg);







static inline void nrf_twi_frequency_set(NRF_TWI_Type * p_reg,
                                           nrf_twi_frequency_t frequency);
# 284 "../../../../../../modules/nrfx/hal/nrf_twi.h"
static inline uint32_t nrf_twi_errorsrc_get_and_clear(NRF_TWI_Type * p_reg);







static inline void nrf_twi_address_set(NRF_TWI_Type * p_reg, uint8_t address);
# 301 "../../../../../../modules/nrfx/hal/nrf_twi.h"
static inline uint8_t nrf_twi_rxd_get(NRF_TWI_Type * p_reg);







static inline void nrf_twi_txd_set(NRF_TWI_Type * p_reg, uint8_t data);







static inline void nrf_twi_shorts_set(NRF_TWI_Type * p_reg,
                                        uint32_t mask);



static inline void nrf_twi_task_trigger(NRF_TWI_Type * p_reg,
                                          nrf_twi_task_t task)
{
    *((volatile uint32_t *)((uint8_t *)p_reg + (uint32_t)task)) = 0x1UL;
}

static inline uint32_t * nrf_twi_task_address_get(NRF_TWI_Type * p_reg,
                                                    nrf_twi_task_t task)
{
    return (uint32_t *)((uint8_t *)p_reg + (uint32_t)task);
}

static inline void nrf_twi_event_clear(NRF_TWI_Type * p_reg,
                                         nrf_twi_event_t event)
{
    *((volatile uint32_t *)((uint8_t *)p_reg + (uint32_t)event)) = 0x0UL;

    volatile uint32_t dummy = *((volatile uint32_t *)((uint8_t *)p_reg + (uint32_t)event));
    (void)dummy;

}

static inline 
# 344 "../../../../../../modules/nrfx/hal/nrf_twi.h" 3 4
               _Bool 
# 344 "../../../../../../modules/nrfx/hal/nrf_twi.h"
                    nrf_twi_event_check(NRF_TWI_Type * p_reg,
                                         nrf_twi_event_t event)
{
    return (
# 347 "../../../../../../modules/nrfx/hal/nrf_twi.h" 3 4
           _Bool
# 347 "../../../../../../modules/nrfx/hal/nrf_twi.h"
               )*(volatile uint32_t *)((uint8_t *)p_reg + (uint32_t)event);
}

static inline uint32_t * nrf_twi_event_address_get(NRF_TWI_Type * p_reg,
                                                     nrf_twi_event_t event)
{
    return (uint32_t *)((uint8_t *)p_reg + (uint32_t)event);
}

static inline void nrf_twi_shorts_enable(NRF_TWI_Type * p_reg,
                                           uint32_t mask)
{
    p_reg->SHORTS |= mask;
}

static inline void nrf_twi_shorts_disable(NRF_TWI_Type * p_reg,
                                            uint32_t mask)
{
    p_reg->SHORTS &= ~(mask);
}

static inline void nrf_twi_int_enable(NRF_TWI_Type * p_reg,
                                        uint32_t mask)
{
    p_reg->INTENSET = mask;
}

static inline void nrf_twi_int_disable(NRF_TWI_Type * p_reg,
                                         uint32_t mask)
{
    p_reg->INTENCLR = mask;
}

static inline 
# 380 "../../../../../../modules/nrfx/hal/nrf_twi.h" 3 4
               _Bool 
# 380 "../../../../../../modules/nrfx/hal/nrf_twi.h"
                    nrf_twi_int_enable_check(NRF_TWI_Type * p_reg,
                                              nrf_twi_int_mask_t twi_int)
{
    return (
# 383 "../../../../../../modules/nrfx/hal/nrf_twi.h" 3 4
           _Bool
# 383 "../../../../../../modules/nrfx/hal/nrf_twi.h"
               )(p_reg->INTENSET & twi_int);
}

static inline void nrf_twi_enable(NRF_TWI_Type * p_reg)
{
    p_reg->ENABLE = ((5UL) << (0UL));
}

static inline void nrf_twi_disable(NRF_TWI_Type * p_reg)
{
    p_reg->ENABLE = ((0UL) << (0UL));
}

static inline void nrf_twi_pins_set(NRF_TWI_Type * p_reg,
                                      uint32_t scl_pin,
                                      uint32_t sda_pin)
{



    p_reg->PSELSCL = scl_pin;





    p_reg->PSELSDA = sda_pin;

}

static inline uint32_t nrf_twi_scl_pin_get(NRF_TWI_Type * p_reg)
{



    return p_reg->PSELSCL;

}

static inline uint32_t nrf_twi_sda_pin_get(NRF_TWI_Type * p_reg)
{



    return p_reg->PSELSDA;

}

static inline void nrf_twi_frequency_set(NRF_TWI_Type * p_reg,
                                           nrf_twi_frequency_t frequency)
{
    p_reg->FREQUENCY = frequency;
}

static inline uint32_t nrf_twi_errorsrc_get_and_clear(NRF_TWI_Type * p_reg)
{
    uint32_t error_source = p_reg->ERRORSRC;


    p_reg->ERRORSRC = error_source;

    return error_source;
}

static inline void nrf_twi_address_set(NRF_TWI_Type * p_reg, uint8_t address)
{
    p_reg->ADDRESS = address;
}

static inline uint8_t nrf_twi_rxd_get(NRF_TWI_Type * p_reg)
{
    return (uint8_t)p_reg->RXD;
}

static inline void nrf_twi_txd_set(NRF_TWI_Type * p_reg, uint8_t data)
{
    p_reg->TXD = data;
}

static inline void nrf_twi_shorts_set(NRF_TWI_Type * p_reg,
                                        uint32_t mask)
{
    p_reg->SHORTS = mask;
}
# 47 "../../../../../../modules/nrfx/drivers/include/nrfx_twi.h" 2
# 62 "../../../../../../modules/nrfx/drivers/include/nrfx_twi.h"
typedef struct
{
    NRF_TWI_Type * p_twi;
    uint8_t drv_inst_idx;
} nrfx_twi_t;
# 76 "../../../../../../modules/nrfx/drivers/include/nrfx_twi.h"
enum {

    NRFX_TWI0_INST_IDX,




    NRFX_TWI_ENABLED_COUNT
};



typedef struct
{
    uint32_t scl;
    uint32_t sda;
    nrf_twi_frequency_t frequency;
    uint8_t interrupt_priority;
    
# 94 "../../../../../../modules/nrfx/drivers/include/nrfx_twi.h" 3 4
   _Bool 
# 94 "../../../../../../modules/nrfx/drivers/include/nrfx_twi.h"
                       hold_bus_uninit;
} nrfx_twi_config_t;
# 115 "../../../../../../modules/nrfx/drivers/include/nrfx_twi.h"
typedef enum
{
    NRFX_TWI_EVT_DONE,
    NRFX_TWI_EVT_ADDRESS_NACK,
    NRFX_TWI_EVT_DATA_NACK,
    NRFX_TWI_EVT_OVERRUN,
    NRFX_TWI_EVT_BUS_ERROR
} nrfx_twi_evt_type_t;


typedef enum
{
    NRFX_TWI_XFER_TX,
    NRFX_TWI_XFER_RX,
    NRFX_TWI_XFER_TXRX,
    NRFX_TWI_XFER_TXTX
} nrfx_twi_xfer_type_t;


typedef struct
{
    nrfx_twi_xfer_type_t type;
    uint8_t address;
    size_t primary_length;
    size_t secondary_length;
    uint8_t * p_primary_buf;
    uint8_t * p_secondary_buf;
} nrfx_twi_xfer_desc_t;
# 190 "../../../../../../modules/nrfx/drivers/include/nrfx_twi.h"
typedef struct
{
    nrfx_twi_evt_type_t type;
    nrfx_twi_xfer_desc_t xfer_desc;
} nrfx_twi_evt_t;


typedef void (* nrfx_twi_evt_handler_t)(nrfx_twi_evt_t const * p_event,
                                        void * p_context);
# 215 "../../../../../../modules/nrfx/drivers/include/nrfx_twi.h"
nrfx_err_t nrfx_twi_init(nrfx_twi_t const * p_instance,
                         nrfx_twi_config_t const * p_config,
                         nrfx_twi_evt_handler_t event_handler,
                         void * p_context);






void nrfx_twi_uninit(nrfx_twi_t const * p_instance);






void nrfx_twi_enable(nrfx_twi_t const * p_instance);






void nrfx_twi_disable(nrfx_twi_t const * p_instance);
# 266 "../../../../../../modules/nrfx/drivers/include/nrfx_twi.h"
nrfx_err_t nrfx_twi_tx(nrfx_twi_t const * p_instance,
                       uint8_t address,
                       uint8_t const * p_data,
                       size_t length,
                       
# 270 "../../../../../../modules/nrfx/drivers/include/nrfx_twi.h" 3 4
                      _Bool 
# 270 "../../../../../../modules/nrfx/drivers/include/nrfx_twi.h"
                                         no_stop);
# 295 "../../../../../../modules/nrfx/drivers/include/nrfx_twi.h"
nrfx_err_t nrfx_twi_rx(nrfx_twi_t const * p_instance,
                       uint8_t address,
                       uint8_t * p_data,
                       size_t length);
# 337 "../../../../../../modules/nrfx/drivers/include/nrfx_twi.h"
nrfx_err_t nrfx_twi_xfer(nrfx_twi_t const * p_instance,
                         nrfx_twi_xfer_desc_t const * p_xfer_desc,
                         uint32_t flags);
# 349 "../../../../../../modules/nrfx/drivers/include/nrfx_twi.h"

# 349 "../../../../../../modules/nrfx/drivers/include/nrfx_twi.h" 3 4
_Bool 
# 349 "../../../../../../modules/nrfx/drivers/include/nrfx_twi.h"
    nrfx_twi_is_busy(nrfx_twi_t const * p_instance);
# 358 "../../../../../../modules/nrfx/drivers/include/nrfx_twi.h"
size_t nrfx_twi_data_count_get(nrfx_twi_t const * const p_instance);
# 370 "../../../../../../modules/nrfx/drivers/include/nrfx_twi.h"
uint32_t nrfx_twi_stopped_event_get(nrfx_twi_t const * p_instance);
# 387 "../../../../../../modules/nrfx/drivers/include/nrfx_twi.h"
static inline nrfx_err_t nrfx_twi_bus_recover(uint32_t scl_pin, uint32_t sda_pin);


static inline nrfx_err_t nrfx_twi_bus_recover(uint32_t scl_pin, uint32_t sda_pin)
{
    return nrfx_twi_twim_bus_recover(scl_pin, sda_pin);
}





void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void);
void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler(void);
# 64 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 2
# 101 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
typedef struct
{
    uint8_t inst_idx;
    union
    {

        nrfx_twim_t twim;


        nrfx_twi_t twi;

    } u;
    
# 113 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
   _Bool 
# 113 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
           use_easy_dma;
} nrf_drv_twi_t;
# 139 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
typedef enum
{
    NRF_DRV_TWI_FREQ_100K = NRF_TWI_FREQ_100K ,
    NRF_DRV_TWI_FREQ_250K = NRF_TWI_FREQ_250K ,
    NRF_DRV_TWI_FREQ_400K = NRF_TWI_FREQ_400K
} nrf_drv_twi_frequency_t;




typedef struct
{
    uint32_t scl;
    uint32_t sda;
    nrf_drv_twi_frequency_t frequency;
    uint8_t interrupt_priority;
    
# 155 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
   _Bool 
# 155 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
                           clear_bus_init;
    
# 156 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
   _Bool 
# 156 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
                           hold_bus_uninit;
} nrf_drv_twi_config_t;
# 182 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
typedef enum
{
    NRF_DRV_TWI_EVT_DONE,
    NRF_DRV_TWI_EVT_ADDRESS_NACK,
    NRF_DRV_TWI_EVT_DATA_NACK
} nrf_drv_twi_evt_type_t;




typedef enum
{
    NRF_DRV_TWI_XFER_TX,
    NRF_DRV_TWI_XFER_RX,
    NRF_DRV_TWI_XFER_TXRX,
    NRF_DRV_TWI_XFER_TXTX
} nrf_drv_twi_xfer_type_t;




typedef struct
{
    nrf_drv_twi_xfer_type_t type;
    uint8_t address;
    uint8_t primary_length;
    uint8_t secondary_length;
    uint8_t * p_primary_buf;
    uint8_t * p_secondary_buf;
} nrf_drv_twi_xfer_desc_t;
# 257 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
typedef struct
{
    nrf_drv_twi_evt_type_t type;
    nrf_drv_twi_xfer_desc_t xfer_desc;
} nrf_drv_twi_evt_t;




typedef void (* nrf_drv_twi_evt_handler_t)(nrf_drv_twi_evt_t const * p_event,
                                           void * p_context);
# 284 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
ret_code_t nrf_drv_twi_init(nrf_drv_twi_t const * p_instance,
                            nrf_drv_twi_config_t const * p_config,
                            nrf_drv_twi_evt_handler_t event_handler,
                            void * p_context);






static inline
void nrf_drv_twi_uninit(nrf_drv_twi_t const * p_instance);






static inline
void nrf_drv_twi_enable(nrf_drv_twi_t const * p_instance);






static inline
void nrf_drv_twi_disable(nrf_drv_twi_t const * p_instance);
# 334 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
static inline
ret_code_t nrf_drv_twi_tx(nrf_drv_twi_t const * p_instance,
                          uint8_t address,
                          uint8_t const * p_data,
                          uint8_t length,
                          
# 339 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
                         _Bool 
# 339 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
                                               no_stop);
# 359 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
static inline
ret_code_t nrf_drv_twi_rx(nrf_drv_twi_t const * p_instance,
                          uint8_t address,
                          uint8_t * p_data,
                          uint8_t length);
# 414 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
static inline
ret_code_t nrf_drv_twi_xfer(nrf_drv_twi_t const * p_instance,
                            nrf_drv_twi_xfer_desc_t const * p_xfer_desc,
                            uint32_t flags);
# 427 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
static inline

# 428 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
_Bool 
# 428 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
    nrf_drv_twi_is_busy(nrf_drv_twi_t const * p_instance);
# 439 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
static inline
uint32_t nrf_drv_twi_data_count_get(nrf_drv_twi_t const * const p_instance);
# 453 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
static inline
uint32_t nrf_drv_twi_start_task_get(nrf_drv_twi_t const * p_instance, nrf_drv_twi_xfer_type_t xfer_type);
# 466 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
static inline
uint32_t nrf_drv_twi_stopped_event_get(nrf_drv_twi_t const * p_instance);
# 495 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
static inline
void nrf_drv_twi_uninit(nrf_drv_twi_t const * p_instance)
{
    if (
# 498 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
       0
# 498 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
                           )
    {
        nrfx_twim_uninit(&p_instance->u.twim);
    }
    else if ((!
# 502 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
            0
# 502 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
            ))
    {
        nrfx_twi_uninit(&p_instance->u.twi);
    }
}

static inline
void nrf_drv_twi_enable(nrf_drv_twi_t const * p_instance)
{
    if (
# 511 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
       0
# 511 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
                           )
    {
        nrfx_twim_enable(&p_instance->u.twim);
    }
    else if ((!
# 515 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
            0
# 515 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
            ))
    {
        nrfx_twi_enable(&p_instance->u.twi);
    }
}

static inline
void nrf_drv_twi_disable(nrf_drv_twi_t const * p_instance)
{
    if (
# 524 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
       0
# 524 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
                           )
    {
        nrfx_twim_disable(&p_instance->u.twim);
    }
    else if ((!
# 528 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
            0
# 528 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
            ))
    {
        nrfx_twi_disable(&p_instance->u.twi);
    }
}

static inline
ret_code_t nrf_drv_twi_tx(nrf_drv_twi_t const * p_instance,
                          uint8_t address,
                          uint8_t const * p_data,
                          uint8_t length,
                          
# 539 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
                         _Bool 
# 539 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
                                               no_stop)
{
    ret_code_t result = 0;
    if (
# 542 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
       0
# 542 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
                           )
    {
        result = nrfx_twim_tx(&p_instance->u.twim,
                                address, p_data, length, no_stop);
    }
    else if ((!
# 547 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
            0
# 547 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
            ))
    {
        result = nrfx_twi_tx(&p_instance->u.twi,
                               address, p_data, length, no_stop);
    }
    return result;
}

static inline
ret_code_t nrf_drv_twi_tx_extended(nrf_drv_twi_t const * p_instance,
                                  uint8_t address,
                                  uint8_t const * p_data,
                                  size_t length,
                                  
# 560 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
                                 _Bool 
# 560 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
                                                       no_stop)
{
    ret_code_t result = 0;
    if (
# 563 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
       0
# 563 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
                           )
    {
        result = nrfx_twim_tx(&p_instance->u.twim,
                                address, p_data, length, no_stop);
    }
    else if ((!
# 568 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
            0
# 568 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
            ))
    {
        result = nrfx_twi_tx(&p_instance->u.twi,
                               address, p_data, length, no_stop);
    }
    return result;
}

static inline
ret_code_t nrf_drv_twi_rx(nrf_drv_twi_t const * p_instance,
                          uint8_t address,
                          uint8_t * p_data,
                          uint8_t length)
{
    ret_code_t result = 0;
    if (
# 583 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
       0
# 583 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
                           )
    {
        result = nrfx_twim_rx(&p_instance->u.twim,
                                address, p_data, length);
    }
    else if ((!
# 588 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
            0
# 588 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
            ))
    {
        result = nrfx_twi_rx(&p_instance->u.twi,
                               address, p_data, length);
    }
    return result;
}

static inline
ret_code_t nrf_drv_twi_xfer(nrf_drv_twi_t const * p_instance,
                            nrf_drv_twi_xfer_desc_t const * p_xfer_desc,
                            uint32_t flags)
{
    ret_code_t result = 0;
    if (
# 602 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
       0
# 602 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
                           )
    {

        nrfx_twim_xfer_desc_t const twim_xfer_desc =
        {
            .type = (nrfx_twim_xfer_type_t)p_xfer_desc->type,
            .address = p_xfer_desc->address,
            .primary_length = p_xfer_desc->primary_length,
            .secondary_length = p_xfer_desc->secondary_length,
            .p_primary_buf = p_xfer_desc->p_primary_buf,
            .p_secondary_buf = p_xfer_desc->p_secondary_buf,
        };
        result = nrfx_twim_xfer(&p_instance->u.twim, &twim_xfer_desc, flags);

    }
    else if ((!
# 617 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
            0
# 617 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
            ))
    {

        nrfx_twi_xfer_desc_t const twi_xfer_desc =
        {
            .type = (nrfx_twi_xfer_type_t)p_xfer_desc->type,
            .address = p_xfer_desc->address,
            .primary_length = p_xfer_desc->primary_length,
            .secondary_length = p_xfer_desc->secondary_length,
            .p_primary_buf = p_xfer_desc->p_primary_buf,
            .p_secondary_buf = p_xfer_desc->p_secondary_buf,
        };
        result = nrfx_twi_xfer(&p_instance->u.twi, &twi_xfer_desc, flags);

    }
    return result;
}

static inline

# 636 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
_Bool 
# 636 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
    nrf_drv_twi_is_busy(nrf_drv_twi_t const * p_instance)
{
    
# 638 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
   _Bool 
# 638 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
        result = 0;
    if (
# 639 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
       0
# 639 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
                           )
    {
        result = nrfx_twim_is_busy(&p_instance->u.twim);
    }
    else if ((!
# 643 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
            0
# 643 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
            ))
    {
        result = nrfx_twi_is_busy(&p_instance->u.twi);
    }
    return result;
}

static inline
uint32_t nrf_drv_twi_data_count_get(nrf_drv_twi_t const * const p_instance)
{
    uint32_t result = 0;
    if (
# 654 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
       0
# 654 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
                           )
    {
        if (0) { if (
# 656 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
       0
# 656 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
       ) { } else { assert_nrf_callback((uint16_t)656, (uint8_t *)"../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"); } };
        result = 0;
    }
    else if ((!
# 659 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
            0
# 659 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
            ))
    {
        result = nrfx_twi_data_count_get(&p_instance->u.twi);
    }
    return result;
}

static inline
uint32_t nrf_drv_twi_start_task_get(nrf_drv_twi_t const * p_instance,
                                    nrf_drv_twi_xfer_type_t xfer_type)
{
    uint32_t result = 0;
    if (
# 671 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
       0
# 671 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
                           )
    {
        result = nrfx_twim_start_task_get(&p_instance->u.twim,
                                          (nrfx_twim_xfer_type_t)xfer_type);
    }
    else if ((!
# 676 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
            0
# 676 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
            ))
    {
        if (0) { if (
# 678 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
       0
# 678 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
       ) { } else { assert_nrf_callback((uint16_t)678, (uint8_t *)"../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"); } };
        result = 0;
    }
    return result;
}

static inline
uint32_t nrf_drv_twi_stopped_event_get(nrf_drv_twi_t const * p_instance)
{
    uint32_t result = 0;
    if (
# 688 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
       0
# 688 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
                           )
    {
        result = nrfx_twim_stopped_event_get(&p_instance->u.twim);
    }
    else if ((!
# 692 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h" 3 4
            0
# 692 "../../../../../../integration/nrfx/legacy/nrf_drv_twi.h"
            ))
    {
        result = nrfx_twi_stopped_event_get(&p_instance->u.twi);
    }
    return result;
}
# 7 "C:\\Users\\haris\\Downloads\\DeviceDownload\\nRF5_SDK_17.1.0_ddde560\\nRF5_SDK_17.1.0_ddde560\\examples\\My Project\\adafruit_tag_reader\\nfc_spk_eeprom\\nfc_spk_eeprom.h" 2
# 1 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h" 1
# 204 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h"
typedef struct
{
    uint8_t sens_res[2];
    uint8_t sel_res;
    uint8_t nfc_id_len;
    uint8_t nfc_id[10];
} nfc_a_tag_info;
# 226 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h"
ret_code_t adafruit_pn532_init(nrf_drv_twi_t *p_twi_master, 
# 226 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h" 3 4
                                                           _Bool 
# 226 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h"
                                                                force);
# 235 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h"
ret_code_t adafruit_pn532_i2c_create(void);
# 253 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h"
ret_code_t adafruit_pn532_sam_config(uint8_t mode);






ret_code_t adafruit_pn532_power_down(void);






ret_code_t adafruit_pn532_wake_up(void);
# 276 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h"
ret_code_t adafruit_pn532_firmware_version_get(uint32_t * p_response);
# 287 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h"
ret_code_t adafruit_pn532_cmd_send(uint8_t * p_cmd, uint8_t cmd_len, uint16_t timeout);






ret_code_t adafruit_pn532_field_on(void);






ret_code_t adafruit_pn532_field_off(void);
# 333 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h"
ret_code_t adafruit_pn532_nfc_a_target_init(nfc_a_tag_info * p_tag_info,
                                            uint16_t timeout);
# 349 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h"
ret_code_t adafruit_pn532_in_data_exchange(uint8_t * p_send,
                                           uint8_t send_len,
                                           uint8_t * p_response,
                                           uint8_t * p_response_len);
# 365 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h"
ret_code_t adafruit_pn532_passive_activation_retries_set(uint8_t max_retries);
# 387 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h"
ret_code_t adafruit_pn532_tag2_read(uint8_t start_page, uint8_t * p_buffer);
# 402 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h"
ret_code_t adafruit_pn532_tag2_page_write(uint8_t page, uint8_t * p_data);
# 418 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h"
ret_code_t adafruit_pn532_ndef_uri_tag2_write(uint8_t uri_id, char * p_url, uint8_t data_len);
# 433 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h"
void adafruit_pn532_tag_info_printout(nfc_a_tag_info const * const p_tag_info);
# 447 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h"

# 447 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h" 3 4
_Bool 
# 447 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h"
    adafruit_pn532_is_ready(void);
# 456 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h"

# 456 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h" 3 4
_Bool 
# 456 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h"
    adafruit_pn532_waitready_ms(uint16_t timeout);





ret_code_t adafruit_pn532_ack_read(void);
# 472 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h"
ret_code_t adafruit_pn532_data_read(uint8_t * p_buff, uint8_t n);
# 485 "../../../../../../external/nfc_adafruit_library/adafruit_pn532.h"
ret_code_t adafruit_pn532_command_write(uint8_t * p_cmd, uint8_t cmd_len);
# 8 "C:\\Users\\haris\\Downloads\\DeviceDownload\\nRF5_SDK_17.1.0_ddde560\\nRF5_SDK_17.1.0_ddde560\\examples\\My Project\\adafruit_tag_reader\\nfc_spk_eeprom\\nfc_spk_eeprom.h" 2
# 1 "../../../main.h" 1
# 9 "C:\\Users\\haris\\Downloads\\DeviceDownload\\nRF5_SDK_17.1.0_ddde560\\nRF5_SDK_17.1.0_ddde560\\examples\\My Project\\adafruit_tag_reader\\nfc_spk_eeprom\\nfc_spk_eeprom.h" 2







typedef uint16_t spk_id_t;


typedef uint32_t counter_t;


typedef struct
{
    spk_id_t spk_id;
    counter_t counter;
}spk_t;


typedef struct
{
    uint8_t index_spk_list[5];
    uint8_t count_spk_list;
    spk_t spk_list[5];
    uint8_t nfc_id_len;
    uint8_t nfc_id[10];
}active_nfc_t;


typedef struct
{
    uint8_t index_nfc_list[5];
    uint8_t count_nfc_list;
    active_nfc_t nfc_list[5];
}nfc_spk_reg_t;

static inline uint8_t nfc_spk_start_address_complete_set(uint8_t nfc_index);

static inline uint8_t nfc_spk_start_address_spk_in_complete_set(uint8_t nfc_index, uint8_t spk_index);

static ret_code_t nfc_spk_eeprom_init(nrf_drv_twi_t * const p_twi_master);

static ret_code_t nfc_spk_retrieve_comprehensive();

static ret_code_t nfc_spk_retrieve_complete_set_by_nfc_index(uint8_t nfc_index);

static ret_code_t nfc_spk_retrieve_spk(uint8_t nfc_index, uint8_t spk_index);

static ret_code_t nfc_spk_retrieve_start_page(uint8_t * p_num_of_active_nfc, uint8_t * p_indexes_list);

static ret_code_t nfc_spk_save_comprehensive();

static ret_code_t nfc_spk_save_complete_set_by_nfc_index(uint8_t nfc_index);

static ret_code_t nfc_spk_save_spk(uint8_t nfc_index, uint8_t spk_index);

static void nfc_spk_reset_register();



static void delay_after_write();

static uint8_t nfc_spk_get_nfc_count();
# 2 "C:\\Users\\haris\\Downloads\\DeviceDownload\\nRF5_SDK_17.1.0_ddde560\\nRF5_SDK_17.1.0_ddde560\\examples\\My Project\\adafruit_tag_reader\\nfc_spk_eeprom\\nfc_spk_eeprom.c" 2
# 1 "../../../../../../components/libraries/eeprom/eeprom.h" 1







# 1 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/string.h" 1 3 4
# 56 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/string.h" 3 4

# 56 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 7.10/include/string.h" 3 4
void * (memcpy) (void *__s1, const void *__s2, size_t __n);
void * (memccpy) (void *__s1, const void *__s2, int __c, size_t __n);
void * (mempcpy) (void *__s1, const void *__s2, size_t __n);
void * (memmove) (void *__s1, const void *__s2, size_t __n);
int (memcmp) (const void *__s1, const void *__s2, size_t __n);
size_t (memlcp) (const void *__s1, const void *__s2, size_t __n);
void * (memchr) (const void *__s, int __c, size_t __n);
void * (memrchr) (const void *__s, int __c, size_t __n);
void * (memmem) (const void *__s1, size_t __n1, const void *__s2, size_t __n2);
void * (memset) (void *__s, int __c, size_t __n);
char * (strcpy) (char *__s1, const char *__s2);
char * (stpcpy) (char *__s1, const char *__s2);
char * (strncpy) (char *__s1, const char *__s2, size_t __n);
char * (stpncpy) (char *__s1, const char *__s2, size_t __n);
size_t (strlcpy) (char *__s1, const char *__s2, size_t __n);
char * (strcat) (char *__s1, const char *__s2);
char * (strncat) (char *__s1, const char *__s2, size_t __n);
size_t (strlcat) (char *__s1, const char *__s2, size_t __n);
int (strcmp) (const char *__s1, const char *__s2);
int (strncmp) (const char *__s1, const char *__s2, size_t __n);
int (strcasecmp) (const char *__s1, const char *__s2);
int (strncasecmp) (const char *__s1, const char *__s2, size_t __n);
int (strcoll) (const char *__s1, const char *__s2);
char * (strchr) (const char *__s, int __c);
char * (strnchr) (const char *__str, size_t __n, int __ch);
size_t (strcspn) (const char *__s1, const char *__s2);
char * (strpbrk) (const char *__s1, const char *__s2);
char * (strrchr) (const char *__s, int __c);
size_t (strspn) (const char *__s1, const char *__s2);
char * (strstr) (const char *__s1, const char *__s2);
char * (strnstr) (const char *__s1, const char *__s2, size_t __n);
char * (strcasestr) (const char *__s1, const char *__s2);
char * (strncasestr) (const char *__s1, const char *__s2, size_t __n);
size_t (strlen) (const char *__s);
size_t (strnlen) (const char *__s, size_t __n);
size_t (strxfrm) (char *__s1, const char *__s2, size_t __n);
char * (strtok) (char *__s1, const char *__s2);
char * (strtok_r) (char *__s1, const char *__s2, char **__s3);
char * (strsep) (char **__stringp, const char *__delim);
char * (strdup) (const char *__s1);
char * (strndup) (const char *__s1, size_t __n);
char * (strerror) (int __num);
# 9 "../../../../../../components/libraries/eeprom/eeprom.h" 2
# 56 "../../../../../../components/libraries/eeprom/eeprom.h"

# 56 "../../../../../../components/libraries/eeprom/eeprom.h"
typedef struct{
  uint8_t p_data[32];
  uint8_t length;
}eeprom_data;

typedef struct{
  
# 62 "../../../../../../components/libraries/eeprom/eeprom.h" 3 4
 _Bool 
# 62 "../../../../../../components/libraries/eeprom/eeprom.h"
      read;
  eeprom_data data;
}eeprom;

ret_code_t eeprom_init(nrf_drv_twi_t * const p_twi_master);

ret_code_t eeprom_write_data(eeprom_data *p_eeprom_data, uint16_t start_address_data);

ret_code_t eeprom_read_data(eeprom_data *p_eeprom_data, uint16_t start_address_data, uint8_t length);
# 3 "C:\\Users\\haris\\Downloads\\DeviceDownload\\nRF5_SDK_17.1.0_ddde560\\nRF5_SDK_17.1.0_ddde560\\examples\\My Project\\adafruit_tag_reader\\nfc_spk_eeprom\\nfc_spk_eeprom.c" 2
# 1 "../../../../../../components/libraries/log/nrf_log.h" 1
# 52 "../../../../../../components/libraries/log/nrf_log.h"
# 1 "../../../../../../components/libraries/util/sdk_common.h" 1
# 59 "../../../../../../components/libraries/util/sdk_common.h"
# 1 "../../../../../../components/libraries/util/sdk_os.h" 1
# 60 "../../../../../../components/libraries/util/sdk_common.h" 2

# 1 "../../../../../../components/libraries/util/app_util.h" 1
# 62 "../../../../../../components/libraries/util/sdk_common.h" 2
# 1 "../../../../../../components/libraries/util/sdk_macros.h" 1
# 63 "../../../../../../components/libraries/util/sdk_common.h" 2
# 53 "../../../../../../components/libraries/log/nrf_log.h" 2
# 1 "../../../../../../components/libraries/experimental_section_vars/nrf_section.h" 1
# 54 "../../../../../../components/libraries/log/nrf_log.h" 2

# 1 "../../../../../../components/libraries/strerror/nrf_strerror.h" 1
# 69 "../../../../../../components/libraries/strerror/nrf_strerror.h"
char const * nrf_strerror_get(ret_code_t code);
# 81 "../../../../../../components/libraries/strerror/nrf_strerror.h"
char const * nrf_strerror_find(ret_code_t code);
# 56 "../../../../../../components/libraries/log/nrf_log.h" 2
# 81 "../../../../../../components/libraries/log/nrf_log.h"
# 1 "../../../../../../components/libraries/log/src/nrf_log_internal.h" 1
# 44 "../../../../../../components/libraries/log/src/nrf_log_internal.h"
# 1 "../../../../../../components/drivers_nrf/nrf_soc_nosd/nrf_error.h" 1
# 45 "../../../../../../components/libraries/log/src/nrf_log_internal.h" 2



# 1 "../../../../../../components/libraries/log/nrf_log_instance.h" 1
# 45 "../../../../../../components/libraries/log/nrf_log_instance.h"
# 1 "../../../../../../components/libraries/log/nrf_log_types.h" 1
# 48 "../../../../../../components/libraries/log/nrf_log_types.h"
typedef enum
{
    NRF_LOG_SEVERITY_NONE,
    NRF_LOG_SEVERITY_ERROR,
    NRF_LOG_SEVERITY_WARNING,
    NRF_LOG_SEVERITY_INFO,
    NRF_LOG_SEVERITY_DEBUG,
    NRF_LOG_SEVERITY_INFO_RAW,
} nrf_log_severity_t;






typedef struct
{
    uint16_t order_idx;
    uint16_t filter;
} nrf_log_module_dynamic_data_t;






typedef struct
{
    uint32_t filter_lvls;
} nrf_log_module_filter_data_t;






typedef struct
{
    const char * p_module_name;
    uint8_t info_color_id;
    uint8_t debug_color_id;
    nrf_log_severity_t compiled_lvl;
    nrf_log_severity_t initial_lvl;
} nrf_log_module_const_data_t;
# 46 "../../../../../../components/libraries/log/nrf_log_instance.h" 2
# 76 "../../../../../../components/libraries/log/nrf_log_instance.h"
extern nrf_log_module_dynamic_data_t * __start_log_dynamic_data; extern void * __stop_log_dynamic_data;
extern nrf_log_module_filter_data_t * __start_log_filter_data; extern void * __stop_log_filter_data;
extern nrf_log_module_const_data_t * __start_log_const_data; extern void * __stop_log_const_data;
# 49 "../../../../../../components/libraries/log/src/nrf_log_internal.h" 2
# 1 "../../../../../../components/libraries/log/nrf_log_types.h" 1
# 50 "../../../../../../components/libraries/log/src/nrf_log_internal.h" 2
# 310 "../../../../../../components/libraries/log/src/nrf_log_internal.h"
extern nrf_log_module_dynamic_data_t m_nrf_log_app_logs_data_dynamic;
extern const nrf_log_module_const_data_t m_nrf_log_app_logs_data_const;
# 363 "../../../../../../components/libraries/log/src/nrf_log_internal.h"
typedef struct
{
    uint32_t type : 2;
    uint32_t in_progress: 1;
    uint32_t data : 29;
} nrf_log_generic_header_t;

typedef struct
{
    uint32_t type : 2;
    uint32_t in_progress: 1;
    uint32_t severity : 3;
    uint32_t nargs : 4;
    uint32_t addr : 22;
} nrf_log_std_header_t;

typedef struct
{
    uint32_t type : 2;
    uint32_t in_progress: 1;
    uint32_t severity : 3;
    uint32_t offset : 10;
    uint32_t reserved : 6;
    uint32_t len : 10;
} nrf_log_hexdump_header_t;

typedef union
{
    nrf_log_generic_header_t generic;
    nrf_log_std_header_t std;
    nrf_log_hexdump_header_t hexdump;
    uint32_t raw;
} nrf_log_main_header_t;

typedef struct
{
    nrf_log_main_header_t base;
    uint16_t module_id;
    uint16_t dropped;
    uint32_t timestamp;
} nrf_log_header_t;
# 414 "../../../../../../components/libraries/log/src/nrf_log_internal.h"
void nrf_log_frontend_std_0(uint32_t severity_mid, char const * const p_str);
# 423 "../../../../../../components/libraries/log/src/nrf_log_internal.h"
void nrf_log_frontend_std_1(uint32_t severity_mid,
                            char const * const p_str,
                            uint32_t val0);
# 434 "../../../../../../components/libraries/log/src/nrf_log_internal.h"
void nrf_log_frontend_std_2(uint32_t severity_mid,
                            char const * const p_str,
                            uint32_t val0,
                            uint32_t val1);
# 446 "../../../../../../components/libraries/log/src/nrf_log_internal.h"
void nrf_log_frontend_std_3(uint32_t severity_mid,
                            char const * const p_str,
                            uint32_t val0,
                            uint32_t val1,
                            uint32_t val2);
# 459 "../../../../../../components/libraries/log/src/nrf_log_internal.h"
void nrf_log_frontend_std_4(uint32_t severity_mid,
                            char const * const p_str,
                            uint32_t val0,
                            uint32_t val1,
                            uint32_t val2,
                            uint32_t val3);
# 473 "../../../../../../components/libraries/log/src/nrf_log_internal.h"
void nrf_log_frontend_std_5(uint32_t severity_mid,
                            char const * const p_str,
                            uint32_t val0,
                            uint32_t val1,
                            uint32_t val2,
                            uint32_t val3,
                            uint32_t val4);
# 488 "../../../../../../components/libraries/log/src/nrf_log_internal.h"
void nrf_log_frontend_std_6(uint32_t severity_mid,
                            char const * const p_str,
                            uint32_t val0,
                            uint32_t val1,
                            uint32_t val2,
                            uint32_t val3,
                            uint32_t val4,
                            uint32_t val5);
# 506 "../../../../../../components/libraries/log/src/nrf_log_internal.h"
void nrf_log_frontend_hexdump(uint32_t severity_mid,
                              const void * const p_data,
                              uint16_t length);






uint8_t nrf_log_getchar(void);
# 82 "../../../../../../components/libraries/log/nrf_log.h" 2
# 262 "../../../../../../components/libraries/log/nrf_log.h"
char const * nrf_log_push(char * const p_str);
# 4 "C:\\Users\\haris\\Downloads\\DeviceDownload\\nRF5_SDK_17.1.0_ddde560\\nRF5_SDK_17.1.0_ddde560\\examples\\My Project\\adafruit_tag_reader\\nfc_spk_eeprom\\nfc_spk_eeprom.c" 2
# 1 "../../../../../../components/libraries/delay/nrf_delay.h" 1
# 64 "../../../../../../components/libraries/delay/nrf_delay.h"
static inline void nrf_delay_ms(uint32_t ms_time)
{
    if (ms_time == 0)
    {
        return;
    }

    do {
        nrfx_coredep_delay_us(1000);
    } while (--ms_time);
}
# 5 "C:\\Users\\haris\\Downloads\\DeviceDownload\\nRF5_SDK_17.1.0_ddde560\\nRF5_SDK_17.1.0_ddde560\\examples\\My Project\\adafruit_tag_reader\\nfc_spk_eeprom\\nfc_spk_eeprom.c" 2
# 44 "C:\\Users\\haris\\Downloads\\DeviceDownload\\nRF5_SDK_17.1.0_ddde560\\nRF5_SDK_17.1.0_ddde560\\examples\\My Project\\adafruit_tag_reader\\nfc_spk_eeprom\\nfc_spk_eeprom.c"
static nfc_spk_reg_t m_reg;

static inline uint8_t nfc_spk_start_address_complete_set(uint8_t nfc_index)
{
    return 0x0CC0 +nfc_index*(1+5)+1;
# 57 "C:\\Users\\haris\\Downloads\\DeviceDownload\\nRF5_SDK_17.1.0_ddde560\\nRF5_SDK_17.1.0_ddde560\\examples\\My Project\\adafruit_tag_reader\\nfc_spk_eeprom\\nfc_spk_eeprom.c"
}

static inline uint8_t nfc_spk_start_address_spk_in_complete_set(uint8_t nfc_index, uint8_t spk_index)
{
    return nfc_spk_start_address_complete_set(nfc_index)+spk_index+1;
# 70 "C:\\Users\\haris\\Downloads\\DeviceDownload\\nRF5_SDK_17.1.0_ddde560\\nRF5_SDK_17.1.0_ddde560\\examples\\My Project\\adafruit_tag_reader\\nfc_spk_eeprom\\nfc_spk_eeprom.c"
}

static ret_code_t nfc_spk_eeprom_init(nrf_drv_twi_t * const p_twi_master)
{
    ret_code_t err;

    if (1 && (3 >= NRF_LOG_SEVERITY_INFO) && (NRF_LOG_SEVERITY_INFO <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_INFO) { nrf_log_frontend_std_0(((NRF_LOG_SEVERITY_INFO) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "Initializing EEPROM"); } };
    err = eeprom_init(p_twi_master);
    if (err != ((0x0) + 0))
    {
        if (1 && (3 >= NRF_LOG_SEVERITY_ERROR) && (NRF_LOG_SEVERITY_ERROR <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_ERROR) { nrf_log_frontend_std_1(((NRF_LOG_SEVERITY_ERROR) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "Failed init eeprom in function %s!", (uint32_t)(__func__)); } };
        return err;
    }

    if ((0x0CC0 % 32) != 0)
    {
        if (1 && (3 >= NRF_LOG_SEVERITY_ERROR) && (NRF_LOG_SEVERITY_ERROR <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_ERROR) { nrf_log_frontend_std_0(((NRF_LOG_SEVERITY_ERROR) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "We temporarily doesn't support start address for data to be not divisible by 32, " "since it doesn't represent the start address of any page of EEPROM it will make our flow for" "storing/retrieving data to/from eeprom screwed"); } }

                                                                       ;
        return ((0x0) + 16);
    }

    return ((0x0) + 0);
}

static ret_code_t nfc_spk_retrieve_comprehensive()
{
    ret_code_t err;



    if (1 && (3 >= NRF_LOG_SEVERITY_INFO) && (NRF_LOG_SEVERITY_INFO <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_INFO) { nrf_log_frontend_std_0(((NRF_LOG_SEVERITY_INFO) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "Trying to retrieve comprehensive SPK and NFC"); } };

    err = nfc_spk_retrieve_start_page(&m_reg.count_nfc_list, m_reg.index_nfc_list);
    if (err != ((0x0) + 0))
    {
        if (1 && (3 >= NRF_LOG_SEVERITY_ERROR) && (NRF_LOG_SEVERITY_ERROR <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_ERROR) { nrf_log_frontend_std_1(((NRF_LOG_SEVERITY_ERROR) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "Error while retrieving start page in function %s", (uint32_t)(__func__)); } };
        return err;
    }

    for (int i = 0; i < m_reg.count_nfc_list; i++)
    {
        err = nfc_spk_retrieve_complete_set_by_nfc_index(m_reg.index_nfc_list[i]);
        if (err != ((0x0) + 0))
        {
            if (1 && (3 >= NRF_LOG_SEVERITY_ERROR) && (NRF_LOG_SEVERITY_ERROR <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_ERROR) { nrf_log_frontend_std_2(((NRF_LOG_SEVERITY_ERROR) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "Failed to retrieve complete set with NFC index %d in function %s", (uint32_t)(m_reg.index_nfc_list[i]), (uint32_t)(__func__)); } };
            return err;
        }
    }

    return ((0x0) + 0);
}

static ret_code_t nfc_spk_retrieve_complete_set_by_nfc_index(uint8_t nfc_index)
{
    ret_code_t err;
    eeprom_data data;

    if (1 && (3 >= NRF_LOG_SEVERITY_INFO) && (NRF_LOG_SEVERITY_INFO <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_INFO) { nrf_log_frontend_std_1(((NRF_LOG_SEVERITY_INFO) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "Trying to retrieve complete set of NFC index %d", (uint32_t)(nfc_index)); } };
    if (nfc_index < 0 || nfc_index >= 5)
    {
        if (1 && (3 >= NRF_LOG_SEVERITY_ERROR) && (NRF_LOG_SEVERITY_ERROR <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_ERROR) { nrf_log_frontend_std_2(((NRF_LOG_SEVERITY_ERROR) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "The nfc_index %d is not in range of [0, MAX_NFC_COUNT) in function %s", (uint32_t)(nfc_index), (uint32_t)(__func__)); } };
        return ((0x0) + 7);
    }


    err = eeprom_read_data(&data, nfc_spk_start_address_complete_set(nfc_index), 32);
    if (err != ((0x0) + 0))
    {
        if (1 && (3 >= NRF_LOG_SEVERITY_ERROR) && (NRF_LOG_SEVERITY_ERROR <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_ERROR) { nrf_log_frontend_std_1(((NRF_LOG_SEVERITY_ERROR) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "Error when retrieving first page of a complete set in index NFC %d", (uint32_t)(nfc_index)); } };
        return err;
    }
    m_reg.nfc_list[nfc_index].nfc_id_len = data.p_data[0];
    memcpy(m_reg.nfc_list[nfc_index].nfc_id, &(data.p_data[(0 +1)]), m_reg.nfc_list[nfc_index].nfc_id_len);
    m_reg.nfc_list[nfc_index].count_spk_list = data.p_data[(0 +1) + m_reg.nfc_list[nfc_index].nfc_id_len];
    memcpy(m_reg.nfc_list[nfc_index].index_spk_list, &(data.p_data[(0 +1) + m_reg.nfc_list[nfc_index].nfc_id_len + 1]), m_reg.nfc_list[nfc_index].count_spk_list);


    for (int i = 0; i < m_reg.nfc_list[nfc_index].count_spk_list; i++)
    {
        uint8_t index_in_spk_list = m_reg.nfc_list[nfc_index].index_spk_list[i];
        err = nfc_spk_retrieve_spk(nfc_index, index_in_spk_list);
        if (err != ((0x0) + 0))
        {
            if (1 && (3 >= NRF_LOG_SEVERITY_ERROR) && (NRF_LOG_SEVERITY_ERROR <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_ERROR) { nrf_log_frontend_std_2(((NRF_LOG_SEVERITY_ERROR) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "Failed to retreive SPK with NFC index %d in function %s", (uint32_t)(nfc_index), (uint32_t)(__func__)); } };
            return err;
        }
    }

    return ((0x0) + 0);
}

static ret_code_t nfc_spk_retrieve_spk(uint8_t nfc_index, uint8_t spk_index)
{
    ret_code_t err;
    eeprom_data data;

    if (1 && (3 >= NRF_LOG_SEVERITY_INFO) && (NRF_LOG_SEVERITY_INFO <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_INFO) { nrf_log_frontend_std_2(((NRF_LOG_SEVERITY_INFO) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "Trying to retrieve SPK index %d with NFC index %d", (uint32_t)(spk_index), (uint32_t)(nfc_index)); } };
    if (nfc_index < 0 || nfc_index >= 5)
    {
        if (1 && (3 >= NRF_LOG_SEVERITY_ERROR) && (NRF_LOG_SEVERITY_ERROR <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_ERROR) { nrf_log_frontend_std_2(((NRF_LOG_SEVERITY_ERROR) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "The nfc_index %d is not in range of [0, MAX_NFC_COUNT) in function %s", (uint32_t)(nfc_index), (uint32_t)(__func__)); } };
        return ((0x0) + 7);
    }

    if (spk_index < 0 || spk_index >= 5)
    {
        if (1 && (3 >= NRF_LOG_SEVERITY_ERROR) && (NRF_LOG_SEVERITY_ERROR <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_ERROR) { nrf_log_frontend_std_2(((NRF_LOG_SEVERITY_ERROR) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "The spk_index %d is not in range of [0, MAX_SPK_COUNT_PER_NFC) in function %s", (uint32_t)(spk_index), (uint32_t)(__func__)); } };
        return ((0x0) + 7);
    }

    err = eeprom_read_data(&data,
                            nfc_spk_start_address_spk_in_complete_set(nfc_index, spk_index),
                            ((sizeof(spk_id_t))+(sizeof(counter_t))));
    if (err != ((0x0) + 0))
    {
        if (1 && (3 >= NRF_LOG_SEVERITY_ERROR) && (NRF_LOG_SEVERITY_ERROR <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_ERROR) { nrf_log_frontend_std_3(((NRF_LOG_SEVERITY_ERROR) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "Failed to retrieve SPK with index : %d in NFC index : %d, in function %s", (uint32_t)(spk_index), (uint32_t)(nfc_index), (uint32_t)(__func__)); } };
        return err;
    }


    m_reg.nfc_list[nfc_index].spk_list[spk_index].spk_id = (((uint16_t)(&(data.p_data[0]))[0])<<8) | (((uint16_t)(&(data.p_data[0]))[1]));
    m_reg.nfc_list[nfc_index].spk_list[spk_index].counter = (((uint32_t)(&(data.p_data[(0 +(sizeof(spk_id_t)))]))[0])<<24) | (((uint32_t)(&(data.p_data[(0 +(sizeof(spk_id_t)))]))[1])<<16) | (((uint32_t)(&(data.p_data[(0 +(sizeof(spk_id_t)))]))[2])<<8) | (((uint32_t)(&(data.p_data[(0 +(sizeof(spk_id_t)))]))[3]));
    return ((0x0) + 0);
}

static ret_code_t nfc_spk_retrieve_start_page(uint8_t * p_num_of_active_nfc, uint8_t * p_indexes_list)
{
    ret_code_t err;
    eeprom_data data;

    if (1 && (3 >= NRF_LOG_SEVERITY_INFO) && (NRF_LOG_SEVERITY_INFO <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_INFO) { nrf_log_frontend_std_0(((NRF_LOG_SEVERITY_INFO) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "Retrieving the first page!"); } };
    err = eeprom_read_data(&data, 0x0CC0, 1+5);
    if (err != ((0x0) + 0))
    {
        if (1 && (3 >= NRF_LOG_SEVERITY_INFO) && (NRF_LOG_SEVERITY_INFO <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_INFO) { nrf_log_frontend_std_0(((NRF_LOG_SEVERITY_INFO) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "Failed to retreive the first page of NFC SPK"); } };
        return err;
    }
    *p_num_of_active_nfc = data.p_data[0];

    for (int i = 0; i<*p_num_of_active_nfc; i++)
    {
        p_indexes_list[i] = data.p_data[(0 +1)+i];
    }

    return ((0x0) + 0);
}

static ret_code_t nfc_spk_save_comprehensive()
{
    ret_code_t err;

    if (1 && (3 >= NRF_LOG_SEVERITY_INFO) && (NRF_LOG_SEVERITY_INFO <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_INFO) { nrf_log_frontend_std_0(((NRF_LOG_SEVERITY_INFO) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "Trying to write to eeprom comprehensively"); } };
    for(int i = 0; i < m_reg.count_nfc_list; i++)
    {
        uint8_t index_in_nfc_list = m_reg.index_nfc_list[i];
        err = nfc_spk_save_complete_set_by_nfc_index(index_in_nfc_list);
        if (err != ((0x0) + 0))
        {
            if (1 && (3 >= NRF_LOG_SEVERITY_ERROR) && (NRF_LOG_SEVERITY_ERROR <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_ERROR) { nrf_log_frontend_std_2(((NRF_LOG_SEVERITY_ERROR) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "Falied to write to eeprom with NFC index %d in function %s", (uint32_t)(index_in_nfc_list), (uint32_t)(__func__)); } };
            return err;
        }
    }
    return ((0x0) + 0);
}

static ret_code_t nfc_spk_save_complete_set_by_nfc_index(uint8_t nfc_index)
{
    ret_code_t err;

    if (1 && (3 >= NRF_LOG_SEVERITY_INFO) && (NRF_LOG_SEVERITY_INFO <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_INFO) { nrf_log_frontend_std_1(((NRF_LOG_SEVERITY_INFO) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "Trying to write complete set of NFC index %d", (uint32_t)(nfc_index)); } };
    if (nfc_index < 0 || nfc_index >= 5)
    {
        if (1 && (3 >= NRF_LOG_SEVERITY_ERROR) && (NRF_LOG_SEVERITY_ERROR <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_ERROR) { nrf_log_frontend_std_2(((NRF_LOG_SEVERITY_ERROR) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "The nfc_index %d is not in range of [0, MAX_NFC_COUNT) in function %s", (uint32_t)(nfc_index), (uint32_t)(__func__)); } };
        return ((0x0) + 7);
    }

    for (int i = 0; i < m_reg.nfc_list[nfc_index].count_spk_list; i++)
    {
        uint8_t index_in_spk_list = m_reg.nfc_list[nfc_index].index_spk_list[i];
        err = nfc_spk_save_spk(nfc_index, index_in_spk_list);
        if (err != ((0x0) + 0))
        {
            if (1 && (3 >= NRF_LOG_SEVERITY_ERROR) && (NRF_LOG_SEVERITY_ERROR <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_ERROR) { nrf_log_frontend_std_2(((NRF_LOG_SEVERITY_ERROR) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "Failed to write to eeprom with NFC index %d in function %s", (uint32_t)(nfc_index), (uint32_t)(__func__)); } };
            return err;
        }
    }

    return ((0x0) + 0);
}

static ret_code_t nfc_spk_save_spk(uint8_t nfc_index, uint8_t spk_index)
{
    ret_code_t err;
    eeprom_data data;

    if (1 && (3 >= NRF_LOG_SEVERITY_INFO) && (NRF_LOG_SEVERITY_INFO <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_INFO) { nrf_log_frontend_std_2(((NRF_LOG_SEVERITY_INFO) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "Trying to write SPK index %d with NFC index %d", (uint32_t)(spk_index), (uint32_t)(nfc_index)); } };
    if (nfc_index < 0 || nfc_index >= 5)
    {
        if (1 && (3 >= NRF_LOG_SEVERITY_ERROR) && (NRF_LOG_SEVERITY_ERROR <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_ERROR) { nrf_log_frontend_std_2(((NRF_LOG_SEVERITY_ERROR) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "The nfc_index %d is not in range of [0, MAX_NFC_COUNT) in function %s", (uint32_t)(nfc_index), (uint32_t)(__func__)); } };
        return ((0x0) + 7);
    }

    if (spk_index < 0 || spk_index >= 5)
    {
        if (1 && (3 >= NRF_LOG_SEVERITY_ERROR) && (NRF_LOG_SEVERITY_ERROR <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_ERROR) { nrf_log_frontend_std_2(((NRF_LOG_SEVERITY_ERROR) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "The spk_index %d is not in range of [0, MAX_SPK_COUNT_PER_NFC) in function %s", (uint32_t)(spk_index), (uint32_t)(__func__)); } };
        return ((0x0) + 7);
    }


    (&(data.p_data[0]))[0] = ((uint8_t)(m_reg.nfc_list[nfc_index].spk_list[spk_index].spk_id>>8)); (&(data.p_data[0]))[1] = ((uint8_t)m_reg.nfc_list[nfc_index].spk_list[spk_index].spk_id);
    (&(data.p_data[(0 +(sizeof(spk_id_t)))]))[0] = ((uint8_t)(m_reg.nfc_list[nfc_index].spk_list[spk_index].counter>>24)); (&(data.p_data[(0 +(sizeof(spk_id_t)))]))[1] = ((uint8_t)(m_reg.nfc_list[nfc_index].spk_list[spk_index].counter>>16)); (&(data.p_data[(0 +(sizeof(spk_id_t)))]))[2] = ((uint8_t)(m_reg.nfc_list[nfc_index].spk_list[spk_index].counter>>8)); (&(data.p_data[(0 +(sizeof(spk_id_t)))]))[3] = ((uint8_t)m_reg.nfc_list[nfc_index].spk_list[spk_index].counter);;


    data.length = ((sizeof(spk_id_t))+(sizeof(counter_t)));

    err = eeprom_write_data(&data, nfc_spk_start_address_spk_in_complete_set(nfc_index, spk_index));
    if (err != ((0x0) + 0))
    {
        if (1 && (3 >= NRF_LOG_SEVERITY_ERROR) && (NRF_LOG_SEVERITY_ERROR <= 3)) { if (NRF_LOG_SEVERITY_DEBUG >= NRF_LOG_SEVERITY_ERROR) { nrf_log_frontend_std_3(((NRF_LOG_SEVERITY_ERROR) | (((uint32_t)(&m_nrf_log_app_logs_data_const) - (uint32_t)&__start_log_const_data) / sizeof(nrf_log_module_const_data_t)) << 16), "Failed to write to eeprom with SPK index %d and NFC index %d in function %s", (uint32_t)(spk_index), (uint32_t)(nfc_index), (uint32_t)(__func__)); } };
        return err;
    }

    delay_after_write();
    return ((0x0) + 0);
}
# 336 "C:\\Users\\haris\\Downloads\\DeviceDownload\\nRF5_SDK_17.1.0_ddde560\\nRF5_SDK_17.1.0_ddde560\\examples\\My Project\\adafruit_tag_reader\\nfc_spk_eeprom\\nfc_spk_eeprom.c"
static void nfc_spk_reset_register()
{
    memset(&m_reg, 0, sizeof(nfc_spk_reg_t));
}

static void delay_after_write(){
    nrf_delay_ms(15);
}

static uint8_t get_nfc_count()
{
    return m_reg.count_nfc_list;
}
