package com.example.bikemonitor

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.tooling.preview.Preview
import com.example.bikemonitor.ui.theme.BikeMonitorTheme

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContent {
            BikeMonitorTheme {
                Scaffold(modifier = Modifier.fillMaxSize()) { innerPadding ->
                    Greeting(
                        name = "Android",
                        modifier = Modifier.padding(innerPadding)
                    )
                }
            }
        }
    }
}

@Composable
fun Greeting(name: String, modifier: Modifier = Modifier) {
    Text(
        text = "Hello $name!",
        modifier = modifier
    )
}

@Preview(showBackground = true)
@Composable
fun GreetingPreview() {
    BikeMonitorTheme {
        Greeting("Android")
    }
}

@Composable
fun BikeChartScreen(dataStream: String) {
    // Tutaj stworzysz wykres LineChart
    // dataStream będzie parsowany (split(",")) na: angle, velocity, wheel, pwm

    Column(modifier = Modifier.fillMaxSize()) {
        Text("Odebrane dane: $dataStream", modifier = Modifier.padding(8.dp))

        // Wykres w czasie rzeczywistym
        AndroidView(
            modifier = Modifier.fillMaxSize().weight(1f),
            factory = { context ->
                LineChart(context).apply {
                    // Konfiguracja wykresu (kolory, siatka, skale)
                }
            },
            update = { chart ->
                // Tutaj dodajesz nowe punkty do wykresu
                updateChart(chart, dataStream)
            }
        )
    }
}