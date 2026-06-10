package ai.deepmost.odyx.ui.common

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import ai.deepmost.odyx.ui.theme.OdyxAccent
import ai.deepmost.odyx.ui.theme.OdyxHairline
import ai.deepmost.odyx.ui.theme.OdyxText
import ai.deepmost.odyx.ui.theme.OdyxTextDim

/** Hairline divider. */
@Composable
fun Hairline(modifier: Modifier = Modifier) {
    Box(modifier.fillMaxWidth().height(1.dp).background(OdyxHairline))
}

/** Dense label/value readout row, monospace, value right-aligned. */
@Composable
fun Readout(label: String, value: String, accent: Boolean = false, valueColor: Color? = null) {
    Row(
        Modifier.fillMaxWidth().padding(horizontal = 12.dp, vertical = 2.dp),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(label, color = OdyxTextDim, style = androidx.compose.material3.MaterialTheme.typography.bodySmall)
        Text(
            value,
            color = valueColor ?: if (accent) OdyxAccent else OdyxText,
            fontWeight = if (accent) FontWeight.Bold else FontWeight.Normal,
            style = androidx.compose.material3.MaterialTheme.typography.bodyMedium
        )
    }
}

/** Section header. */
@Composable
fun SectionHeader(title: String) {
    Text(
        title,
        color = OdyxAccent,
        modifier = Modifier.fillMaxWidth().padding(horizontal = 12.dp, vertical = 6.dp),
        style = androidx.compose.material3.MaterialTheme.typography.labelSmall
    )
}

/** Tiny inline sparkline for a value history. */
@Composable
fun Sparkline(data: List<Float>, color: Color, label: String) {
    androidx.compose.foundation.layout.Column(
        Modifier.fillMaxWidth().padding(horizontal = 12.dp, vertical = 2.dp)
    ) {
        Text(label, color = OdyxTextDim,
            style = androidx.compose.material3.MaterialTheme.typography.labelSmall)
        androidx.compose.foundation.Canvas(
            Modifier.fillMaxWidth().height(28.dp).background(Color(0xFF101012))
        ) {
            if (data.size < 2) return@Canvas
            val mn = data.min(); val mx = data.max()
            val span = (mx - mn).takeIf { it > 1e-6f } ?: 1f
            val dx = size.width / (data.size - 1)
            var prev = Offset(0f, size.height * (1f - (data[0] - mn) / span))
            for (i in 1 until data.size) {
                val x = i * dx
                val y = size.height * (1f - (data[i] - mn) / span)
                drawLine(color, prev, Offset(x, y), strokeWidth = 1.5f)
                prev = Offset(x, y)
            }
        }
    }
}

fun f(v: Double, dp: Int = 2): String = "%.${dp}f".format(v)
fun f(v: Float, dp: Int = 2): String = "%.${dp}f".format(v)
fun v3(a: FloatArray, dp: Int = 4): String =
    if (a.size >= 3) "${f(a[0], dp)} ${f(a[1], dp)} ${f(a[2], dp)}" else "--"
