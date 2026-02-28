#!/usr/bin/env python3
from pathlib import Path
import re

ROOT = Path('C:/uavNetSim')
md_in = ROOT / 'docs' / 'UavNetSim_documentation.md'
md_out = ROOT / 'docs' / 'UavNetSim_documentation_zh_full.md'

text = md_in.read_text(encoding='utf-8')

# Replace file headings
text = re.sub(r"## File: (.+)", r"## 文件: \1", text)
# Class and Function headings
text = re.sub(r"### Class `(\w+)`  \(line (\d+)\)", r"### 类 `\1`（行 \2）", text)
text = re.sub(r"### Function `(\w+)([^`]*)`  \(line (\d+)\)", r"### 函数 `\1\2`（行 \3）", text)
# Replace Docstring/Methods/Calls/Parameters/Returns
text = text.replace('**Docstring:**', '**文档字符串：**')
text = text.replace('**Methods:**', '**方法：**')
text = text.replace('**Calls:**', '**调用：**')
text = re.sub(r"- Calls: ", r"    - 调用：", text)
text = text.replace('Parameters:', '参数：')
text = text.replace('Returns:', '返回：')
text = text.replace('**Methods:**', '**方法：**')

# Replace other common labels
text = text.replace('**Docstring:**', '**文档字符串：**')
text = text.replace('**Calls:**', '**调用：**')

md_out.write_text(text, encoding='utf-8')
print('Wrote', md_out)
