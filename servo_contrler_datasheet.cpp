struct Position {
  int open;
  int close;
};

constexpr Position servo_pos[] = {
    {3242, 2264}, // S1
    {2047, 3272}, // S2
    {3261, 1858}, // S3
    {0, 4095},    // S4
    {949, 1681},  // S5
    {2834, 2102}, // S6
};

constexpr int servo1_plat_close = 1017;
constexpr int servo1_block_close = 2264;

enum class MechaCommand {
  NONE,
  PLANT_HOLD,             // プラント把持
  PLANT_RELEASE,          // プラント解放
  BLOCK_HOLD_AND_LIFT_UP, // ブロック全把持・上昇
  F_BLOCK_RELEASE,        // 前方ブロックの解放
  R_BLOCK_RELEASE,        // 後方ブロックの解放
  BLOCK_LIFT_DOWN,        // 全ブロック下降
  UP_ARM,                 // アーム上昇 ←キャリブレーション待ち
  DOWN_ARM,               // アーム下降←キャリブレーション待ち
  RISE_ARM,               // アームを起き上がらせる
  CLOSE_ARM,              // アームを閉じる
  OPEN_ARM,               // アームを開ける
};
///*
// 〇・・・プラント把持or開放
// △・・・ブロック回収アームの把持&ブロック回収機構全体の昇降
// □・・・前方ブロック回収アームの開放
// ×・・・後方ブロック回収アームの開放
// コントローラー中心の大きなボタン・・・ブロック回収機構全体の降下
// 右スティック・・・左右でロボットの回転。上下方向の入力はなし
// 右スティック＋R1・・・上下でアームの上下。左右方向の入力はなし ←キャリブレーション待ち
// 左スティック・・・機体の前後左右移動
// L1・・・アーム全体を起き上がらせる
// L2・・・アームハンドを閉じる
// R2・・・アームハンドを開く

// 十字下・・・スタート時に向いていた方向を向く
// 上・・・スタートと反対方向
// 右・・・スタート時から見て右に90回転した方向
// 左・・・左に90度
///

FeetechPositionControl BLOCK_HOLDER_6(uart5, 6, 2834); // 2250-3236
FeetechPositionControl BLOCK_HOLDER_5(uart5, 5, 949);  // 542-1680
FeetechPositionControl BLOCK_LIFTER_4(uart5, 4, 4095); // 523-4000
FeetechPositionControl PLANT_HOLDER_3(uart5, 3, 3261); // 1123-3261
FeetechPositionControl RAIL_REVO_2(uart5, 2, 2047);    // 525-3272
FeetechPositionControl BLOCK_PUTTER_1(uart5, 1, 3242); // 579-3022