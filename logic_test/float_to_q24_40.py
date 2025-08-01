def float_to_q24_40(x: float) -> str:
    """
    将十进制浮点数转换为 Q24.40 有符号定点数（十六进制表示）
    Q24.40 共 64 位，整数部分 24 位，分数部分 40 位。
    """
    # 定点放大倍数（相当于左移 40 位）
    scale = 1 << 40
    # 转换为整数
    fixed_int = int(round(x * scale))

    # 保证在 64 位有符号数范围内
    if fixed_int >= (1 << 63):
        raise OverflowError("数值溢出：超过 Q24.40 最大值")
    if fixed_int < -(1 << 63):
        raise OverflowError("数值溢出：小于 Q24.40 最小值")

    # 转换为 64 位有符号十六进制（补码形式）
    hex_str = hex((fixed_int + (1 << 64)) % (1 << 64))
    return "0x" + hex_str[2:].zfill(16)  # 补齐到16位

# 示例用法
test_vals = [-1.9994, 0.9994, 0.4999e-7, 0.9997e-7]
for val in test_vals:
    print(f"{val} -> {float_to_q24_40(val)}")