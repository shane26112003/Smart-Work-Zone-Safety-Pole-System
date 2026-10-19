#!/usr/bin/env python3
"""
Generates a beautifully formatted Microsoft Word (.docx) document containing
all 5 research paper metric tables with academic styling, bold headers,
subtle shading, and professional borders.

Output: research_paper_tables.docx
"""

import os
import docx
from docx import Document
from docx.shared import Inches, Pt, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT, WD_ALIGN_VERTICAL
from docx.oxml import OxmlElement, parse_xml
from docx.oxml.ns import nsdecls, qn

OUTPUT_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "research_paper_tables.docx")


def set_cell_background(cell, hex_color):
    """Sets background shading color for a table cell."""
    tcPr = cell._element.get_or_add_tcPr()
    shd = parse_xml(f'<w:shd {nsdecls("w")} w:fill="{hex_color}"/>')
    tcPr.append(shd)


def set_cell_margins(cell, top=100, bottom=100, left=150, right=150):
    """Sets internal padding for a cell."""
    tcPr = cell._element.get_or_add_tcPr()
    tcMar = parse_xml(
        f'<w:tcMar {nsdecls("w")}>'
        f'<w:top w:w="{top}" w:type="dxa"/>'
        f'<w:bottom w:w="{bottom}" w:type="dxa"/>'
        f'<w:left w:w="{left}" w:type="dxa"/>'
        f'<w:right w:w="{right}" w:type="dxa"/>'
        f'</w:tcMar>'
    )
    tcPr.append(tcMar)


def set_table_borders(table, color="D3D3D3", sz="4", val="single"):
    """Applies clean horizontal borders similar to academic booktabs."""
    tblPr = table._element.xpath('w:tblPr')
    if tblPr:
        borders = parse_xml(
            f'<w:tblBorders {nsdecls("w")}>'
            f'<w:top w:val="{val}" w:sz="12" w:space="0" w:color="1B365D"/>'
            f'<w:bottom w:val="{val}" w:sz="12" w:space="0" w:color="1B365D"/>'
            f'<w:insideH w:val="{val}" w:sz="{sz}" w:space="0" w:color="{color}"/>'
            f'<w:insideV w:val="none"/>'
            f'<w:left w:val="none"/>'
            f'<w:right w:val="none"/>'
            f'</w:tblBorders>'
        )
        tblPr[0].append(borders)


def format_table(table, col_widths, headers, data_rows, highlight_rows=None):
    """Populates and styles a table with headers and data."""
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = False

    # Header Row
    hdr_cells = table.rows[0].cells
    for i, title in enumerate(headers):
        hdr_cells[i].text = title
        set_cell_background(hdr_cells[i], "1B365D") # Navy blue
        set_cell_margins(hdr_cells[i], top=140, bottom=140, left=140, right=140)
        p = hdr_cells[i].paragraphs[0]
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER if i > 0 else WD_ALIGN_PARAGRAPH.LEFT
        for run in p.runs:
            run.font.name = "Times New Roman"
            run.font.size = Pt(10)
            run.font.bold = True
            run.font.color.rgb = RGBColor(255, 255, 255)

    # Data Rows
    for r_idx, row_data in enumerate(data_rows):
        row_cells = table.add_row().cells
        is_highlight = highlight_rows and (r_idx in highlight_rows)
        bg_color = "EBF3FA" if is_highlight else ("F8F9FA" if r_idx % 2 == 1 else "FFFFFF")

        for c_idx, val in enumerate(row_data):
            row_cells[c_idx].text = str(val)
            set_cell_background(row_cells[c_idx], bg_color)
            set_cell_margins(row_cells[c_idx], top=100, bottom=100, left=140, right=140)
            p = row_cells[c_idx].paragraphs[0]
            p.alignment = WD_ALIGN_PARAGRAPH.CENTER if c_idx > 0 else WD_ALIGN_PARAGRAPH.LEFT
            for run in p.runs:
                run.font.name = "Times New Roman"
                run.font.size = Pt(9.5)
                if is_highlight:
                    run.font.bold = True
                    run.font.color.rgb = RGBColor(27, 54, 93)
                else:
                    run.font.color.rgb = RGBColor(30, 30, 30)

    # Set Column Widths
    for row in table.rows:
        for idx, width in enumerate(col_widths):
            row.cells[idx].width = width

    set_table_borders(table)


def build_research_tables_doc():
    doc = Document()

    # Set 1-inch margins
    sections = doc.sections
    for section in sections:
        section.top_margin = Inches(1)
        section.bottom_margin = Inches(1)
        section.left_margin = Inches(1)
        section.right_margin = Inches(1)

    # Document Title
    title = doc.add_paragraph()
    title_run = title.add_run("Smart Work-Zone Safety Pole System: Research Paper Metric Tables")
    title_run.font.name = "Times New Roman"
    title_run.font.size = Pt(16)
    title_run.font.bold = True
    title_run.font.color.rgb = RGBColor(27, 54, 93)
    title.alignment = WD_ALIGN_PARAGRAPH.CENTER

    sub = doc.add_paragraph()
    sub_run = sub.add_run("Empirical Results, Sensor Fusion Accuracy, and Performance Evaluations")
    sub_run.font.name = "Times New Roman"
    sub_run.font.size = Pt(11)
    sub_run.font.italic = True
    sub.alignment = WD_ALIGN_PARAGRAPH.CENTER

    doc.add_paragraph() # Spacer

    # ==================== TABLE 1 ====================
    p1 = doc.add_paragraph()
    r1 = p1.add_run("TABLE I\nOBJECT DETECTION AND PPE CLASSIFICATION PERFORMANCE (YOLOv8s)")
    r1.font.name = "Times New Roman"
    r1.font.size = Pt(10.5)
    r1.font.bold = True
    p1.alignment = WD_ALIGN_PARAGRAPH.CENTER

    t1_headers = ["Evaluation Metric", "Exact Value", "Percentage (%)", "Benchmark Target"]
    t1_rows = [
        ["Precision (P)", "0.8702", "87.02%", "> 80.0%"],
        ["Recall (R)", "0.9203", "92.03%", "> 85.0%"],
        ["F1-Score", "0.8945", "89.45%", "> 80.0%"],
        ["mAP@50 (IoU = 0.50)", "0.9469", "94.69%", "> 85.0%"],
        ["mAP@50-95 (COCO Metric)", "0.7193", "71.93%", "> 50.0%"],
        ["PPE Safety Vest Verification", "0.9500", "95.00%", "> 80.0%"],
        ["Civilian Clothing False Positive Rate", "0.0000", "0.00%", "< 5.0%"],
        ["Small False Vehicle Rejection Rate", "1.0000", "100.00%", "100.0%"],
        ["Total Network Layers", "72 layers (fused)", "—", "—"],
        ["Trainable Parameters", "11,156,544", "—", "—"],
        ["Computational Complexity", "28.6 GFLOPs", "—", "—"]
    ]
    t1 = doc.add_table(rows=1, cols=4)
    format_table(t1, [Inches(2.5), Inches(1.2), Inches(1.3), Inches(1.5)], t1_headers, t1_rows, highlight_rows=[3, 4, 5])

    doc.add_paragraph() # Spacer

    # ==================== TABLE 2 ====================
    p2 = doc.add_paragraph()
    r2 = p2.add_run("TABLE II\nVEHICLE TRAJECTORY PREDICTION ERROR ACROSS HORIZONS (6D CA-EKF MODEL)")
    r2.font.name = "Times New Roman"
    r2.font.size = Pt(10.5)
    r2.font.bold = True
    p2.alignment = WD_ALIGN_PARAGRAPH.CENTER

    t2_headers = ["Scenario / Maneuver Category", "ADE@1.0s (m)", "ADE@2.0s (m)", "ADE@3.0s (m)", "FDE@3.0s (m)"]
    t2_rows = [
        ["Highway Cruise (60 – 110 km/h)", "0.000", "0.000", "0.000", "0.000"],
        ["Work-Zone Lateral Drift / Breach", "0.000", "0.000", "0.000", "0.000"],
        ["Lane Change & Evasive Swerve", "0.060", "0.330", "0.760", "1.900"],
        ["Work-Zone Taper Merging", "0.150", "0.610", "1.320", "3.220"],
        ["Standard Braking (-1.8 to -3.2 m/s²)", "0.685", "2.185", "4.520", "10.875"],
        ["Emergency Braking (-5.0 to -7.0 m/s²)", "1.650", "5.245", "10.850", "26.100"],
        ["Work-Zone Acceleration Exit (+2.0 m/s²)", "1.190", "2.370", "3.950", "8.160"],
        ["Low-Speed Queue Congestion", "0.000", "0.000", "0.000", "0.000"],
        ["Overall CA-EKF Benchmark Average", "0.327", "0.997", "2.028", "4.823"],
        ["Baseline 4D CV-EKF Benchmark Average", "0.378", "1.036", "2.061", "4.848"],
        ["Pillar 1 Relative Error Reduction (%)", "13.5%", "3.8%", "1.6%", "0.5%"],
        ["Miss Rate (FDE@3.0s > 2.0 m)", "35.0% (restricted to extreme braking > 5 m/s²)", "—", "—", "—"]
    ]
    t2 = doc.add_table(rows=1, cols=5)
    format_table(t2, [Inches(2.5), Inches(1.0), Inches(1.0), Inches(1.0), Inches(1.0)], t2_headers, t2_rows, highlight_rows=[8, 10])

    doc.add_paragraph() # Spacer

    # ==================== TABLE 3 ====================
    p3 = doc.add_paragraph()
    r3 = p3.add_run("TABLE III\nLOG-DISTANCE RSSI DISTANCE ESTIMATION AND CROSS-MODAL ASSOCIATION ACCURACY")
    r3.font.name = "Times New Roman"
    r3.font.size = Pt(10.5)
    r3.font.bold = True
    p3.alignment = WD_ALIGN_PARAGRAPH.CENTER

    t3_headers = ["Ground Truth Distance", "Measured RSSI", "Estimated Range", "Mean Absolute Error", "Relative Error (%)"]
    t3_rows = [
        ["1.00 m", "-59.2 ± 1.4 dBm", "1.02 m", "0.21 m", "2.0%"],
        ["2.00 m", "-66.4 ± 1.8 dBm", "2.04 m", "0.38 m", "2.0%"],
        ["3.00 m", "-70.5 ± 2.0 dBm", "3.01 m", "0.55 m", "3.3%"],
        ["5.00 m", "-75.8 ± 2.2 dBm", "5.03 m", "0.86 m", "6.0%"],
        ["8.00 m", "-80.6 ± 2.5 dBm", "7.94 m", "1.32 m", "8.2%"],
        ["12.00 m", "-85.1 ± 2.7 dBm", "12.24 m", "1.95 m", "9.6%"],
        ["Line-of-Sight (LOS) Operational Average", "—", "—", "0.878 m", "5.18%"],
        ["Body-Shielded / NLOS Field Average", "—", "—", "1.820 m", "12.40%"],
        ["Weighted Real-World Average", "—", "—", "1.420 m", "8.79%"],
        ["Camera-Tag Identity Association (LOS)", "99.1% Correct Match", "—", "—", "—"],
        ["Camera-Tag Identity Association (NLOS)", "94.5% Correct Match", "—", "—", "—"]
    ]
    t3 = doc.add_table(rows=1, cols=5)
    format_table(t3, [Inches(1.8), Inches(1.5), Inches(1.1), Inches(1.1), Inches(1.0)], t3_headers, t3_rows, highlight_rows=[6, 8, 9])

    doc.add_paragraph() # Spacer

    # ==================== TABLE 4 ====================
    p4 = doc.add_paragraph()
    r4 = p4.add_run("TABLE IV\nMULTI-SENSOR HARDWARE TELEMETRY SPECIFICATIONS AND LATENCY BREAKDOWN")
    r4.font.name = "Times New Roman"
    r4.font.size = Pt(10.5)
    r4.font.bold = True
    p4.alignment = WD_ALIGN_PARAGRAPH.CENTER

    t4_headers = ["Subsystem Component", "Operational Parameter", "Exact Measured Value", "Unit"]
    t4_rows = [
        ["Road TSD20 LiDAR", "Transmission Frequency", "50.0", "Hz"],
        ["Road TSD20 LiDAR", "Operational Range Envelope", "0.10 to 20.00", "m"],
        ["Road TSD20 LiDAR", "Distance Resolution", "± 0.05", "m"],
        ["Road TSD20 LiDAR", "Packet Checksum Integrity", "100.00", "%"],
        ["Worker Badge (MPU6500)", "IMU Sampling Frequency", "50.0", "Hz"],
        ["Worker Badge (MPU6500)", "Telemetry Broadcast Rate", "25.0", "Hz"],
        ["Worker Badge (MPU6500)", "SVM Resolution", "± 0.02", "g"],
        ["Worker Badge (MPU6500)", "Impact Sensitivity (G ≥ 3.20)", "100.00", "%"],
        ["System Latencies", "Wi-Fi UDP Transit Time (ESP32 → Pi 5)", "3.2 ± 0.8", "ms"],
        ["System Latencies", "Targeted Haptic Directive Dispatch Latency", "< 12.0", "ms"],
        ["System Latencies", "GPIO Strobe & Siren Relay Response", "< 1.5", "ms"],
        ["System Latencies", "Total End-to-End Decision Cycle (Pi 5)", "65 – 85", "ms"],
        ["System Framerate", "Raspberry Pi 5 (ARM Cortex-A76)", "12 – 15", "FPS"],
        ["System Framerate", "Workstation Benchmark (Intel Core i7)", "35.2", "FPS"]
    ]
    t4 = doc.add_table(rows=1, cols=4)
    format_table(t4, [Inches(1.8), Inches(2.6), Inches(1.3), Inches(0.8)], t4_headers, t4_rows, highlight_rows=[8, 9, 11])

    doc.add_paragraph() # Spacer

    # ==================== TABLE 5 ====================
    p5 = doc.add_paragraph()
    r5 = p5.add_run("TABLE V\nSAFETY THREAT STRATIFICATION AND COLLISION ALERT CLASSIFICATION PERFORMANCE")
    r5.font.name = "Times New Roman"
    r5.font.size = Pt(10.5)
    r5.font.bold = True
    p5.alignment = WD_ALIGN_PARAGRAPH.CENTER

    t5_headers = ["Threat Level", "Activation Criteria", "Detection Rate", "False Alarm Rate", "Hardware Reaction"]
    t5_rows = [
        ["SAFE", "TTCeff > 4.0 s, dCPA > 4.0 m", "—", "0.00%", "Green Visual Radar HUD"],
        ["CAUTION", "2.5 s < TTCeff ≤ 4.0 s, dCPA ≤ 4.0 m", "100.00%", "0.00%", "Amber Directional Strobe"],
        ["HIGH RISK", "1.5 s < TTCeff ≤ 2.5 s, dCPA ≤ 2.5 m", "100.00%", "0.00%", "Amber/Red Strobe + Buzzer"],
        ["CRITICAL", "TTCeff ≤ 1.5 s or Barrier Breach", "100.00%", "0.00%", "120 dB Siren + Red Strobe"],
        ["Worker Fall Escalation", "Impact G ≥ 3.20 or Freefall G ≤ 0.35 (Mfall = 1.65)", "100.00%", "0.00%", "Targeted Haptic Evacuation"],
        ["TTC Estimation Accuracy", "Mean Absolute Error (MAE_TTC)", "± 0.12 s", "—", "High Precision Closure"]
    ]
    t5 = doc.add_table(rows=1, cols=5)
    format_table(t5, [Inches(1.2), Inches(2.1), Inches(1.1), Inches(1.0), Inches(1.6)], t5_headers, t5_rows, highlight_rows=[3, 4])

    doc.save(OUTPUT_FILE)
    print(f"[SUCCESS] Research paper tables successfully generated: {OUTPUT_FILE}")


if __name__ == "__main__":
    build_research_tables_doc()
