#! /usr/bin/env python
import streamlit as st
import subprocess
import os


def main():

    st.sidebar.title("プログラム選択")
    program_desc = st.sidebar.radio(
        "",
        [
            "単純なCSMA/CAプログラム",
            "状態情報の予測を行わないプログラム",
            "状態情報を予測する情報転送手段プログラム",
            "状態情報の選別を行うプログラム",
        ],
    )
    if program_desc == "単純なCSMA/CAプログラム":
        program_choice = "consensus-csma.out"
    elif program_desc == "状態情報の予測を行わないプログラム":
        program_choice = "consensus-transfer1.out"
    elif program_desc == "状態情報を予測する情報転送手段プログラム":
        program_choice = "consensus-transfer2.out"
    elif program_desc == "状態情報の選別を行うプログラム":
        program_choice = "consensus-transfer3.out"

    # スライダーを作成してパラメータを取得
    num_robots = st.slider(
        "ロボットの数",
        min_value=20,
        max_value=100,
        step=10,
    )
    control_interval = st.radio(
        "ロボットの制御周期",
        options=[0.01, 0.1],
        index=1,
        format_func=lambda x: f"{x} [s]",
    )
    data_length_values = [32, 64, 128, 256]
    data_length = st.select_slider(
        "パケット長[byte]", options=data_length_values, value=64
    )
    contention_window_values = [15, 31, 63, 127, 255, 511, 1023]
    contention_window = st.select_slider(
        "コンテンションウィンドウ", options=contention_window_values, value=127
    )
    num_sim = st.slider(
        "シミュレーション回数", min_value=0, max_value=1000, step=100, value=100
    )

    # 実行ボタン
    if st.button("実行"):
        run_cpp_program(
            program_choice,
            num_robots,
            control_interval,
            data_length,
            contention_window,
            num_sim,
        )


# C++プログラムを実行する関数
def run_cpp_program(
    program_choice,
    num_robots,
    control_interval,
    data_length,
    contention_window,
    num_sim,
):
    # C++プログラムをコマンドラインから実行する
    # args配列の値をコマンドライン引数として渡す
    cpp_program_path = "./consensus-csma.out"

    try:
        # C++プログラムの実行ファイルパスを指定
        cpp_program_path = os.path.join(os.getcwd(), program_choice)

        # コマンドライン引数を作成
        args = [
            str(num_robots),
            str(control_interval),
            str(data_length),
            str(contention_window),
            str(num_sim),
        ]

        # C++プログラムを実行し、結果を取得
        result = subprocess.run(
            [cpp_program_path] + args, capture_output=True, text=True
        )

        # 標準出力を取得
        stdout = result.stdout
        # 標準エラー出力を取得
        stderr = result.stderr

        # 結果を処理する
        # 例: Streamlitで標準出力を表示する
        st.code(stdout)

        # 例: 標準エラー出力がある場合は、エラーメッセージを表示する
        if stderr:
            st.error(f"エラー: {stderr}")

    except Exception as e:
        st.error(f"エラー: {e}")


if __name__ == "__main__":
    main()
