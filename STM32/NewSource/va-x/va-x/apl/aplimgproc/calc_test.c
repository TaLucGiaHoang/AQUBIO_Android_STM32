#include <math.h>
#include <t_stddef.h>

#define AUTH_SIZE_X		80
#define AUTH_SIZE_Y		40
#define DEV_NUM			4
#define X_SEARCH		3
#define Y_SEARCH		5
#define HI_END			219

typedef uint8_t UB;

int match_pos[2][DEV_NUM][2];								//[SOBEL][分割][x,y]=[2][4][2]
int match_score[2][DEV_NUM][2*Y_SEARCH+1][2*X_SEARCH+1];	//[SOBEL][分割][サーチ範囲y][サーチ範囲x]=[2][4][11][7]
int match_soukan[2][DEV_NUM];								//[SOBEL][分割]=[2][4]

UB CapImgBuf[2][2][AUTH_SIZE_X*AUTH_SIZE_Y];				//[登録数][ソーベル][認証サイズ]



/*==========================================================================*/
//	エリア内認証演算
/*==========================================================================*/
static double two_matcher7v( int sizex, int sizey, int cap_num, int sv_num, int stx, int edx, int offsetx, int offsety, int sbl, int learn_num)
{
	volatile int x, y, pos1, pos2;
	volatile long count, dat1, dat2;
	volatile long sum1, sum2, ave1, ave2;
	volatile long sum_mul, sqr1, sqr2;
//	volatile unsigned long addr1, addr2;
	volatile double score;
	
	count = 0;
	sum1 = sum2 = 0;

	for( y = offsety ; y < sizey+offsety ; y++ ){
		for( x = stx+offsetx ; x < edx+offsetx ; x++){

			if( x < stx || x >= edx || y < 0 || y >= sizey ) continue;

			pos1 = y * sizex + x;
			pos2 = (y - offsety) * sizex + (x - offsetx);
			dat1 = CapImgBuf[0][sbl][pos1];
			dat2 = CapImgBuf[1][sbl][pos2];

			//20160909Miya 認証アップ
			if( dat1 > HI_END || dat2 > HI_END)	continue;

			sum1 += dat1;
			sum2 += dat2;
			++count;
		}
	}
	ave1 = sum1 / count;
	ave2 = sum2 / count;
	
	sum_mul = sqr1 = sqr2 = 0;
	
	for( y = offsety ; y < sizey+offsety ; y++ ){
		for( x = stx+offsetx ; x < edx+offsetx ; x++){

			if( x < stx || x >= edx || y < 0 || y >= sizey ) continue;

			pos1 = y * sizex + x;
			pos2 = (y - offsety) * sizex + (x - offsetx);
			dat1 = CapImgBuf[0][sbl][pos1];
			dat2 = CapImgBuf[1][sbl][pos2];

			//20160909Miya 認証アップ
			if(dat1 > HI_END || dat2 > HI_END)	continue;

			sum_mul += ( (dat1 - ave1) * (dat2 - ave2) );
			sqr1 += ( (dat1 - ave1) * (dat1 - ave1) );
			sqr2 += ( (dat2 - ave2) * (dat2 - ave2) );
		}
	}
	
	score = 1000.0 * (double)sum_mul / sqrt( (float)sqr1 ) / sqrt( (float)sqr2 );
	
	if( score > 999.0 ){
		score = 999.0;
	}
	if( score < 0.0 ){
		score = -1.0;
	}

	return( score );

}

/*==========================================================================*/
//	2画像の認証処理
/*==========================================================================*/
static double auto_matching( int cap_num, int sv_num, int sbl )
{
//	UB rtn = 0;
	int /* i, j, */ k, x, y;
	int sizex, sizey, dev_sizex;
	int stx, edx, offsetx, offsety, learn_num;
	int min1;
	volatile double score, max1, ave1 /*, score2 */;
	
	sizex = AUTH_SIZE_X;	//80
	sizey = AUTH_SIZE_Y;	//40
	dev_sizex = sizex / DEV_NUM;	//4分割 80 / 4 = 20

	learn_num = 0;
	min1 = 999;
	ave1 = 0.0;

	for(k = 0 ; k < DEV_NUM ; k++){
		max1 = -100.0;
		match_pos[sbl][k][0] = 0;
		match_pos[sbl][k][1] = 0;
		for( y = -Y_SEARCH ; y <= Y_SEARCH ; y++ ){		
			for( x = -X_SEARCH ; x <= X_SEARCH ; x++ ){
				stx = k * dev_sizex;
				edx = stx + dev_sizex;
				offsetx = x;
				offsety = y;
				//スコアー算出
				score = two_matcher7v( sizex, sizey, cap_num, sv_num, stx, edx, offsetx, offsety, sbl, learn_num );

				match_score[sbl][k][y + Y_SEARCH][x + X_SEARCH] = score;
				if( score >= max1 ){
					max1 = score;
					match_pos[sbl][k][0] = x;
					match_pos[sbl][k][1] = y;
				}
			}
		}
		if( max1 < 0.0 ){
			match_soukan[sbl][k] = -1;
		}else{
			match_soukan[sbl][k] = (int)max1;
		}
		if( (int)max1 <= min1 ){
			min1 = (int)max1;
		}
		ave1 += match_soukan[sbl][k];
		
	}

	ave1 = ave1 / 4.0;

	return(ave1);
}

void calc_test_main(void)
{
	int	cap_num, sv_num, r1, r2;
	int x, y, cnt, dat;
//	double score1, score2;

	r1 = r2 = 0;
	cap_num = 0;	//キャプチャーした画像番号
	sv_num = 1;		//登録されている画像番号

	cnt = 0;
	for(y = 0 ; y < AUTH_SIZE_Y ; y++){
		dat = 80;
		for(x = 0 ; x < AUTH_SIZE_X ; x++){
			CapImgBuf[0][r1][cnt] = (UB)dat;
			CapImgBuf[0][r2][cnt] = (UB)dat;
			++cnt; ++dat;
		}
	}

	cnt = 0;
	for(y = 0 ; y < AUTH_SIZE_Y ; y++){
		dat = 100;
		for(x = 0 ; x < AUTH_SIZE_X ; x++){
			CapImgBuf[1][r1][cnt] = (UB)dat;
			CapImgBuf[1][r2][cnt] = (UB)dat;
			++cnt; ++dat;
		}
	}

	//memcpy( &CapImgBuf[0][r1][0], &g_ubSobelR1Buf[0], AUTH_SIZE_X * AUTH_SIZE_Y );		//キャプチャー画像->テンプレート0番
	//memcpy( &CapImgBuf[0][r2][0], &g_ubSobelR2Buf[0], AUTH_SIZE_X * AUTH_SIZE_Y );		//キャプチャー画像->テンプレート0番

	//memcpy( &CapImgBuf[1][r1][0], &g_ubSobelR1SvBuf[0], AUTH_SIZE_X * AUTH_SIZE_Y );	//保存画像(登録1回目)->テンプレート1番
	/* score1 = */ auto_matching( cap_num, sv_num, r1 /*, &auth_num */);

	//memcpy( &CapImgBuf[1][r2][0], &g_ubSobelR2SvBuf[0], AUTH_SIZE_X * AUTH_SIZE_Y );	//保存画像(登録1回目)->テンプレート1番
	/* score2 = */ auto_matching( cap_num, sv_num, r2 /*, &auth_num */);


}

