# 电子设计竞赛 LaTeX 报告模板

[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/SandOcean-ovo/Template-for-Electrical-Competition-Report)

## 简介

这是一个为全国大学生电子设计竞赛（及其他类似科创竞赛）设计的通用 LaTeX 报告模板。

它提供了一个结构清晰、格式规范的框架，旨在帮助参赛者摆脱繁杂的格式调整，专注于报告内容的撰写，从而快速、高效地完成高质量的设计报告。


## 📁 文件结构

```
TexWork/
├── .gitignore   # Git忽略规则文件，仅跟踪src和pic文件夹
├── README.md    # 模板简介
├── LICENSE      # 许可证内容
├── pic/         # 存放报告中使用的所有图片
└── src/         # 存放 LaTeX 源文件 (.tex)
```

## 🚀 如何使用

### 1. 环境准备

确保您的电脑上已经安装了 LaTeX 发行版，例如：
- [**TeX Live**](https://www.tug.org/texlive/) (Windows, Linux, macOS)
- [**MiKTeX**](https://miktex.org/) (Windows)
- [**MacTeX**](https://www.tug.org/mactex/) (macOS)

推荐使用带有 LaTeX 插件的编辑器，如 **VS Code (配合 LaTeX Workshop 插件)**，以获得更好的编写和编译体验。

### 2. 撰写报告

1.  将您在报告中需要引用的所有图片文件（如系统框图、电路图、流程图、实物图等）统一放入 `pic` 文件夹中。
2.  打开 `src` 文件夹下的 `.tex` 源文件。
3.  根据文件中各个章节的注释提示和 `[占位符]`，填写您自己的项目内容。
4.  在需要插入图片的地方，取消相关代码的注释并修改图片路径，例如：`\includegraphics[width=0.8\linewidth]{../pic/your-image.png}`。 

### 3. 编译生成 PDF

使用您的 LaTeX 编辑器或相关命令行工具编译 `.tex` 文件，即可在项目根目录或指定输出目录生成最终的 PDF 报告。

通常，对于包含中文的文档，建议使用 `XeLaTeX` 或 `pdfLaTex` 引擎进行编译。

## 许可证

本项目采用 MIT 许可证。详情请参阅 [LICENSE](LICENSE) 文件。

---
祝您比赛顺利，取得优异成绩！ 
