■内容の置き換え

○下記の文字列はテンプレート用なので適宜置きかえる

apltmpl.h
apltmpl_task
DTQID_APLTMPL
TSKID_APLTMPL
APLTMPL_EVT_XXX

■公開ヘッダを移動

公開ヘッダを va-x/include に移動する

■他ファイルの変更

○Makefile
  ・VAX_APPL_DIR にディレクトリを追加
  ・VAX_APPL_COBJS にオブジェクトファイルを追加(追加するソースの.cを.oにしたもの)

○va-x.cfg
  ・INCLUDEを追加
------------------------------------------------------------
/* テンプレート */
INCLUDE("apl/apltmpl/apltmpl.cfg");
------------------------------------------------------------

