/*
 * VA-X 認証アルゴリズム
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/01/31 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "mdlauthlib.h"

#include <string.h>
#include <math.h>

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "kernel_cfg.h"

#include "cmndbg.h"
#include "mdlstrg.h"
#include "mdlstrg_data.h"

/*
 * マクロ定義
 */

// デバッグログ
#if 0
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[MDLAUTHLIB]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[MDLAUTHLIB]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[MDLAUTHLIB]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[MDLAUTHLIB]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

#define DISPAUTHLOG	1	//認証ログ出力
#define DISPCALC	0	//演算結果出力


// サービスコールのエラーのログ出力
static inline void svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}
#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))

/*
 * 型定義
 */

#define SIZE_INPUT_X	640		//撮影サイズ(X)
#define SIZE_INPUT_Y	480		//撮影サイズ(Y)
#define	SIZE_VALID_X	480		//切出しサイズ(X)
#define	SIZE_VALID_Y	240		//切出しサイズ(Y)
#define SIZE_PROC_X		120		//処理サイズ(X)　辺1/4(480 / 4 = 120)
#define SIZE_PROC_Y		60		//処理サイズ(Y)　辺1/4(240 / 4 = 60)
#define SIZE_AUTH_X		80		//認証サイズ(X)
#define SIZE_AUTH_Y		40		//認証サイズ(Y)
#define SIZE_SEPA_X		SIZE_AUTH_X / 4		//認証分割サイズ（X) 20
#define SIZE_SEPA_Y		60		//認証分割サイズ（Y)
#define SIZE_MINI_X		30		//極小サイズ(X)　辺1/4(120 / 4 = 30)
#define SIZE_MINI_Y		15		//極小サイズ(Y)　辺1/4( 60 / 4 = 15)
#define SIZE_AUTH_MX	20		//認証MINIサイズ(X)
#define SIZE_AUTH_MY	10		//認証MINIサイズ(Y)


#define R1				1		//R1
#define R2				2		//R2
#define R3				3		//R3
#define R8				8		//全方位
#define RESIZE4			4		//Resize 辺1/4
#define SOUKAN_MATRIX_X	21		//相関マトリックスサイズ(X)
#define SOUKAN_MATRIX_Y	11		//相関マトリックスサイズ(Y)
#define SEPA_SOUKAN_MATRIX_Y	21		//分割相関マトリックスサイズ(Y)
#define SOUKAN_STEP_X	1		//相関ステップ(X pixel)
#define SOUKAN_STEP_Y	1		//相関ステップ(Y pixel)

#define REG_SOUKAN_MATRIX_X	21		//相関マトリックスサイズ(X)
#define REG_SOUKAN_MATRIX_Y	11		//相関マトリックスサイズ(Y)
#define REG_SOUKAN_STEP_X	1		//相関ステップ(X pixel)
#define REG_SOUKAN_STEP_Y	1		//相関ステップ(Y pixel)

#define SOUKAN_MATRIX_MX	3		//相関マトリックスサイズ(X)
#define SOUKAN_MATRIX_MY	3		//相関マトリックスサイズ(Y)
#define SOUKAN_STEP_MX		1		//相関ステップ(X pixel)
#define SOUKAN_STEP_MY		1		//相関ステップ(Y pixel)

//フラグ
static const FLGPTN FLGPTN_MDLSTRG_REQ_COMPLETE =			(0x1 << 0);

// 画像バッファ(80x40)
typedef struct {
	uint8_t	pixels[80 * 40];
} IMAGEBUF_NORMAL_T;

// 画像バッファ(100x40)
typedef struct {
	//uint8_t	pixels[100*40];
	uint8_t	pixels[120 * 60];
} IMAGEBUF_NORNAL_MARGINED_T;

// 画像バッファ(20x10)
typedef struct {
	uint8_t	pixels[20 * 10];
} IMAGEBUF_SMALL_T;

// 画像バッファ(20x10)
typedef struct {
	//uint8_t	pixels[25*10];
	uint8_t	pixels[30 * 15];
} IMAGEBUF_SMALL_MARGINED_T;

// 登録データ (マージンなし)
typedef struct {
	IMAGEBUF_NORMAL_T	original;	// 撮影画像
	IMAGEBUF_NORMAL_T	r1;			// R1データ
	IMAGEBUF_NORMAL_T	r2;			// R2データ
	IMAGEBUF_NORMAL_T	r3;		// R3データ
} AUTHDATA_REGISTERED_T;
static_assert(sizeof(AUTHDATA_REGISTERED_T) == sizeof(MDLAUTHLIB_DATA_REGISTERED_T), "MDLAUTHLIB_DATA_REGISTERED_SIZE");

// 登録データ (小サイズ, マージンなし)
typedef struct {
	IMAGEBUF_SMALL_T	r1;			// R1データ
	IMAGEBUF_SMALL_T	r2;			// R2データ
} AUTHDATA_REGISTERED_SMALL_T;
static_assert(sizeof(AUTHDATA_REGISTERED_SMALL_T) == sizeof(MDLAUTHLIB_DATA_REGISTERED_SMALL_T), "MDLAUTHLIB_DATA_REGISTERED_SMALL_SIZE");

// 認証データ (マージンあり)
typedef struct {
	IMAGEBUF_NORNAL_MARGINED_T	original;	// 撮影画像
	IMAGEBUF_NORNAL_MARGINED_T	r1;			// R1データ
	IMAGEBUF_NORNAL_MARGINED_T	r2;			// R2データ
	IMAGEBUF_NORNAL_MARGINED_T	r3;		// R3データ
	IMAGEBUF_SMALL_MARGINED_T	r1_small;	// R1データ(小サイズ)
	IMAGEBUF_SMALL_MARGINED_T	r2_small;	// R2データ(小サイズ)
} AUTHDATA_CAPTURED_T;
static_assert(sizeof(AUTHDATA_CAPTURED_T) == sizeof(MDLAUTHLIB_DATA_CAPTURED_T), "MDLAUTHLIB_DATA_CAPTURED_SIZE");

// 認証用パラメータ
typedef struct {
	uint16_t	stx1;		//image1 start X
	uint16_t	sty1;		//image1 start Y
	uint16_t	szx1;		//image1 size X
	uint16_t	szy1;		//image1 size Y
	uint16_t	stx2;		//image2 start X
	uint16_t	sty2;		//image2 start Y
	uint16_t	szx2;		//image2 size X
	uint16_t	szy2;		//image2 size Y
	uint16_t	authszx;	//Auth size X
	uint16_t	authszy;	//Auth size Y
	uint16_t	authposix;	//Auth position X
	uint16_t	authposiy;	//Auth position X
	uint64_t	score;		//Auth score
} AUTHDATA_IMAGE_PARA;

// ソーベル処理用ワーク
typedef struct {
    int64_t normalize_array[SIZE_PROC_Y][SIZE_PROC_X];
} WORKBUF_SOBEL_T;

// 登録前登録サイズデータ用ワーク
typedef struct {
	IMAGEBUF_NORMAL_T pixels;
} WORKBUF_REGSIZE_T;

MDLSTRG_DATA_ROOM_STS_T authu_value;


/*
 * 内部関数プロトタイプ
 */
void mdlauthlib_imageproc_resize(AUTHDATA_CAPTURED_T* data, const MDLAUTHLIB_IMAGE_T* input);
void mdlauthlib_imageproc_minisize4(uint8_t sbl, uint16_t inszX, uint16_t inszY, uint16_t outszX, uint16_t outszY, AUTHDATA_CAPTURED_T* data);
void mdlauthlib_imageproc_sobel(uint8_t sbl, uint16_t sizeX, uint16_t sizeY, AUTHDATA_CAPTURED_T* data, WORKBUF_SOBEL_T* work);
void mdlauthlib_imageproc_setregdata(uint64_t stx, uint64_t sty, const AUTHDATA_CAPTURED_T* cap_data, AUTHDATA_REGISTERED_T* reg_data, AUTHDATA_REGISTERED_SMALL_T* reg_mini);
void mdlauthlib_matching_7v(AUTHDATA_IMAGE_PARA *inpara, const uint8_t *img1, const uint8_t *img2);
void mdlauthlib_matching_proc(AUTHDATA_IMAGE_PARA *rsltpara, const uint8_t *img1, const uint8_t *img2);
void mdlauthlib_matching_miniproc(AUTHDATA_IMAGE_PARA *rsltpara, const uint8_t *img1, const uint8_t *img2);
int mdlauthlib_imageproc_finger_search(const MDLAUTHLIB_IMAGE_T* image, const MDLAUTHLIB_RECT_T* rect);
void mdlauthlib_preproc_resize(uint8_t *img, const MDLAUTHLIB_IMAGE_T* input);
void mdlauthlib_preproc_sobel(uint8_t sbl, uint16_t sizeX, uint16_t sizeY, uint8_t *inimg, uint8_t *outimg, WORKBUF_SOBEL_T* work);
void mdlauthlib_preproc_setregdata(uint64_t stx, uint64_t sty, const uint8_t* pre_data, uint8_t* reg_data );
void mdlauthlib_matching_proc_R3(AUTHDATA_IMAGE_PARA *rsltpara, const uint8_t *img1, const uint8_t *img2);
void strg_callback(int event, intptr_t opt1, intptr_t opt2);

/*
 * 定数定義
 */


/*
 * 内部変数
 */


/*
 * 画像の明るさ(輝度平均値)を算出
 */
int mdlauthlib_calc_brightness(const MDLAUTHLIB_IMAGE_T* image, const MDLAUTHLIB_RECT_T* rect)
{
	return 0;
}

/*
 * 画像処理(撮影画像から認証データを生成)
 */
int mdlauthlib_process_image(MDLAUTHLIB_DATA_CAPTURED_T* a_data, MDLAUTHLIB_IMAGE_T* input)
{
	return 0;
}

/*
 * 極小画像のマッチング処理を行いスコアを返す
 */
int mdlauthlib_match_small(const MDLAUTHLIB_DATA_CAPTURED_T* a_captured_data, const MDLAUTHLIB_DATA_REGISTERED_SMALL_T* a_registered_data)
{
	return 0;
}

/*
 * 画像のマッチング処理を行いスコアを返す
 */
void mdlauthlib_match(MDLAUTHLIB_MATCH_RESULT_T* result, const MDLAUTHLIB_DATA_CAPTURED_T* a_captured_data, const MDLAUTHLIB_DATA_REGISTERED_T* a_registered_data)
{
}

/*
 * 撮影データから登録用のデータを生成する
 */
int mdlauthlib_create_data_registered(MDLAUTHLIB_DATA_REGISTERED_T* a_registered_data, MDLAUTHLIB_DATA_REGISTERED_SMALL_T* a_registered_data_small, const MDLAUTHLIB_DATA_CAPTURED_T* a_captured_data, int offset)
{

	return 0;
}

// 指検知用
int mdlauthlib_finger_search(const MDLAUTHLIB_IMAGE_T* a_image, const MDLAUTHLIB_RECT_T* a_rect)
{
	return 0;
}

/*
 * 登録時1回目処理
 */
int mdlauthlib_registered_first_image(MDLAUTHLIB_DATA_CAPTURED_T* a_data, MDLAUTHLIB_IMAGE_T* input)
{

	return 0;
}

/*
 * 登録時2回目処理
 */
int mdlauthlib_registered_second_image(MDLAUTHLIB_DATA_CAPTURED_T* a_data, MDLAUTHLIB_IMAGE_T* input)
{

	return 0;
}

/*
 * 登録画像処理(撮影画像から認証データを生成)
 */
int mdlauthlib_regprocess_image(MDLAUTHLIB_DATA_CAPTURED_T* a_data, MDLAUTHLIB_IMAGE_T* input)
{

	return 0;
}

void center_flash_read(){
}

///////////////////////////////////////////////////////////////////
// 以降、内部関数
// ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
///////////////////////////////////////////////////////////////////
//***************************************************************//
// Date:2018.04.10  M.Miyauchi
// 撮影画像の切り出し→縮小(辺1/4)
// 640 x 480 -> 480 x 240(0, 120) -> 120 x 60
//***************************************************************//
void mdlauthlib_imageproc_resize(AUTHDATA_CAPTURED_T* data, const MDLAUTHLIB_IMAGE_T* input)
{
}

//***************************************************************//
// Date:2018.11.08  maruhashi manabu
// 撮影画像の切り出し→縮小(辺1/4)
// 640 x 480 -> 480 x 240(0, 120) -> 120 x 60
//***************************************************************//
void mdlauthlib_preproc_resize(uint8_t *img, const MDLAUTHLIB_IMAGE_T* input)
{
}

//***************************************************************//
// Date:2018.04.20  M.Miyauchi
// 極小処理(sbl 1:R1 2:R2)
//***************************************************************//
void mdlauthlib_imageproc_minisize4(uint8_t sbl, uint16_t inszX, uint16_t inszY, uint16_t outszX, uint16_t outszY, AUTHDATA_CAPTURED_T* data)
{
}
//***************************************************************//
// Date:2018.04.10  M.Miyauchi
//	add square root normalize 2018.09.18 maruhashi manabu
// Sobel処理(sbl 1:R1 2:R2 3:R3)
//***************************************************************//
void mdlauthlib_imageproc_sobel(uint8_t sbl, uint16_t sizeX, uint16_t sizeY, AUTHDATA_CAPTURED_T* data, WORKBUF_SOBEL_T* work)
{
}

//***************************************************************//
// Date:2018.11.8  maruhashi manabu
// Sobel処理(sbl 3:R3 8:全方向)
//***************************************************************//
void mdlauthlib_preproc_sobel(uint8_t sbl, uint16_t sizeX, uint16_t sizeY, uint8_t *inimg, uint8_t *outimg, WORKBUF_SOBEL_T* work)
{
}

//***************************************************************//
// Date:2018.04.10  M.Miyauchi
// 撮影画像→登録画像
// 120 x 60 -> 80 x 40
//***************************************************************//
void mdlauthlib_imageproc_setregdata(uint64_t stx, uint64_t sty, const AUTHDATA_CAPTURED_T* cap_data, AUTHDATA_REGISTERED_T* reg_data, AUTHDATA_REGISTERED_SMALL_T* reg_mini)
{

}

//***************************************************************//
// Date:2018.11.8  maruhashi manabu
// 撮影画像→登録画像
// 120 x 60 -> 80 x 40
//***************************************************************//
void mdlauthlib_preproc_setregdata(uint64_t stx, uint64_t sty, const uint8_t* pre_data, uint8_t* reg_data )
{
}


//***************************************************************//
// Date:2018.04.10  M.Miyauchi
// Sobel処理(sbl 1:R1 2:R2)
//***************************************************************//
void mdlauthlib_matching_7v(AUTHDATA_IMAGE_PARA *inpara, const uint8_t *img1, const uint8_t *img2)
{
}

/*
 * 認証分割マッチング
 */
void mdlauthlib_matching_proc(AUTHDATA_IMAGE_PARA *rsltpara, const uint8_t *img1, const uint8_t *img2)
{

}

void mdlauthlib_matching_proc_R3(AUTHDATA_IMAGE_PARA *rsltpara, const uint8_t *img1, const uint8_t *img2)
{

}


void mdlauthlib_matching_miniproc(AUTHDATA_IMAGE_PARA *rsltpara, const uint8_t *img1, const uint8_t *img2)
{

//	DBGLOG3("**** SCORE_MINI = %d(%d, %d)\n", (int)max_score, max_posix, max_posiy);
}

// ストレージミドルコールバック
void strg_callback(int event, intptr_t opt1, intptr_t opt2)
{
    DBGLOG1("STRG CALLBACK: %d", event);

    ER er = E_OK;

    switch (event) {
    case MDLSTRG_EVT_REQUEST_COMPLETE:
        er = set_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE);
        assert(er == E_OK);
        break;
    default:
        break;
    }
}

//#define SIZE_INPUT_X	640		//撮影サイズ(X)
//#define SIZE_INPUT_Y	480		//撮影サイズ(Y)
//#define	SIZE_VALID_X	480		//切出しサイズ(X)
//#define	SIZE_VALID_Y	240		//切出しサイズ(Y)
//#define SIZE_PROC_X		120		//処理サイズ(X)　辺1/4(480 / 4 = 120)
//#define SIZE_PROC_Y		60		//処理サイズ(X)　辺1/4(240 / 4 = 60)
//#define SIZE_AUTH_X		80		//処理サイズ(X)
//#define SIZE_AUTH_Y		40		//処理サイズ(Y)
//#define SIZE_MINI_X		30		//極小サイズ(X)　辺1/4(120 / 4 = 30)
//#define SIZE_MINI_Y		15		//極小サイズ(X)　辺1/4( 60 / 4 = 15)


