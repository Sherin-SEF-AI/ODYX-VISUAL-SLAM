package ai.deepmost.odyx.ui.theme

import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Typography
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.sp

// "Operational Materialism": matte near-black surfaces, ONE earned accent.
val OdyxBlack    = Color(0xFF0A0A0B)
val OdyxSurface  = Color(0xFF141416)
val OdyxSurface2 = Color(0xFF1C1C1F)
val OdyxHairline = Color(0xFF2A2A2E)
val OdyxText     = Color(0xFFE6E6E6)
val OdyxTextDim  = Color(0xFF8A8A90)
val OdyxAccent   = Color(0xFFFF7A1A)   // the single accent (active SLAM / KF / loop)
val OdyxGood     = Color(0xFF35C56A)
val OdyxWarn     = Color(0xFFE0B33A)
val OdyxBad      = Color(0xFFD6443B)

private val Scheme = darkColorScheme(
    primary = OdyxAccent,
    onPrimary = OdyxBlack,
    background = OdyxBlack,
    onBackground = OdyxText,
    surface = OdyxSurface,
    onSurface = OdyxText,
    surfaceVariant = OdyxSurface2,
    onSurfaceVariant = OdyxTextDim,
    outline = OdyxHairline,
    error = OdyxBad
)

// Monospace everywhere — SLAM lives on dense numeric readouts.
private val mono = FontFamily.Monospace
private val OdyxTypography = Typography(
    bodyLarge = TextStyle(fontFamily = mono, fontSize = 14.sp),
    bodyMedium = TextStyle(fontFamily = mono, fontSize = 12.sp),
    bodySmall = TextStyle(fontFamily = mono, fontSize = 11.sp),
    labelSmall = TextStyle(fontFamily = mono, fontSize = 10.sp, fontWeight = FontWeight.Medium),
    titleMedium = TextStyle(fontFamily = mono, fontSize = 15.sp, fontWeight = FontWeight.Bold),
    titleLarge = TextStyle(fontFamily = mono, fontSize = 18.sp, fontWeight = FontWeight.Bold)
)

@Composable
fun OdyxTheme(content: @Composable () -> Unit) {
    @Suppress("UNUSED_EXPRESSION") isSystemInDarkTheme()   // ODYX is always dark
    MaterialTheme(colorScheme = Scheme, typography = OdyxTypography, content = content)
}
