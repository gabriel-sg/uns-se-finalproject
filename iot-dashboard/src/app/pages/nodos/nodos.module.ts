import { NgModule } from '@angular/core';

import { ThemeModule } from '../../@theme/theme.module';

import { NodosRoutingModule } from './nodos-routing.module';
import { NodosComponent } from './nodos.component';
import { Nodo1Component } from './nodo1/nodo1.component';
import { Nodo2Component } from './nodo2/nodo2.component';
import { Nodo3Component } from './nodo3/nodo3.component';
import { StatusCardComponent } from '../dashboard/status-card/status-card.component';
import { TemperatureComponent } from '../dashboard/temperature/temperature.component';
import { TemperatureDraggerComponent } from '../dashboard/temperature/temperature-dragger/temperature-dragger.component';
import { SensorDisplayComponent } from './sensor-display/sensor-display.component';
import { NodoCardComponent } from './nodo-card/nodo-card.component';
import { HomeComponent } from './home/home.component';
import { TabComponent } from './tab/tab.component';



@NgModule({
  declarations: [
    NodosComponent,
    Nodo1Component,
    Nodo2Component,
    Nodo3Component,
    StatusCardComponent,
    TemperatureComponent,
    TemperatureDraggerComponent,
    SensorDisplayComponent,
    NodoCardComponent,
    HomeComponent,
    TabComponent,

  ],
  imports: [
    ThemeModule,
    NodosRoutingModule,
  ]
})

export class NodosModule { }
