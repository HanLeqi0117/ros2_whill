/*
MIT License

Copyright (c) 2018 WHILL inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef __ODOM_H__
#define __ODOM_H__

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// オドメトリクラスの型定義
class Odometry
{
  private:
    // ラジアン単位の数値の大きさを制限する： 0 < rad < π
    long double confineRadian(long double rad);

    // オドメトリ情報を格納する構造体
    typedef struct
    {
        double x;
        double y;
        double theta;
    } Space2D;

    // ロボットの車輪の半径
    static constexpr double wheel_radius_ = 0.1325;
    // ロボットの車軸の長さ
    static constexpr double wheel_tread_ = 0.248;

    // 姿勢
    Space2D pose;
    // 速度
    Space2D velocity;

  public:
    // クラスのコンストラクター
    // odom情報の初期化
    Odometry();
    // モータの情報を更新する(モータを回す度呼び出される関数)
    void update(sensor_msgs::msg::JointState joint, double dt);
    // メンバー変数を定義する
    void set(Space2D pose);
    // ロボットの姿勢を初期化する
    void reset();

    // ROSインターフェイス風のオドメトリ情報を取得する
    nav_msgs::msg::Odometry getROSOdometry();
    // 時間スタンプ付きのTF座標変換を取得する関数
    geometry_msgs::msg::TransformStamped getROSTransformStamped();
    // メンバー変数である「pose」をアクセスする
    Space2D getOdom();
};

#endif

