import { Component, OnInit, OnDestroy } from '@angular/core';

import { NbThemeService } from '@nebular/theme'

import { takeWhile } from 'rxjs/operators' ;
import { Observable } from 'rxjs';
import { AngularFireObject } from '@angular/fire/database';
import { FiredbService } from '../services/firedb.service';

interface CardSettings {
  title: string;
  iconClass: string;
  type: string;
  status: boolean;
  actuadorId: string;
}

@Component({
  selector: 'ngx-nodo3',
  templateUrl: './nodo3.component.html',
  styleUrls: ['./nodo3.component.scss']
})

export class Nodo3Component implements OnInit, OnDestroy {
  private alive = true;
  private nodoId = '0x6003';
  private actuadorId1 = 'Actuador1';
  private actuadorId2 = 'Actuador2';
  private actuadorId3 = 'Actuador3';
  private actuadorId4 = 'Actuador4';
  private sensorId1 = 'Sensor1';
  private sensorId2 = 'Sensor2';
  private sensorId3 = 'Sensor3';
  private sensorId4 = 'Sensor4';

  lightCard: CardSettings = {
    title: 'Blue Led',
    iconClass: 'nb-lightbulb',
    type: 'primary',
    status: true,
    actuadorId: this.actuadorId1
  };
  rollerShadesCard: CardSettings = {
    title: 'Green Led',
    iconClass: 'nb-lightbulb',
    type: 'success',
    status: true,
    actuadorId: this.actuadorId2
  };
  wirelessAudioCard: CardSettings = {
    title: 'Wireless Audio',
    iconClass: 'nb-audio',
    type: 'info',
    status: true,
    actuadorId: this.actuadorId3
  };
  coffeeMakerCard: CardSettings = {
    title: 'Coffee Maker',
    iconClass: 'nb-coffee-maker',
    type: 'warning',
    status: true,
    actuadorId: this.actuadorId4
  };

  statusCards: string;

  commonStatusCardsSet: CardSettings[] = [
    this.lightCard,
    this.rollerShadesCard,
    // this.wirelessAudioCard,
    // this.coffeeMakerCard,
  ];

  statusCardsByThemes: {
    default: CardSettings[];
    cosmic: CardSettings[];
    corporate: CardSettings[];
  } = {
    default: this.commonStatusCardsSet,
    cosmic: this.commonStatusCardsSet,
    corporate: [
      {
        ...this.lightCard,
        type: 'warning',
      },
      {
        ...this.rollerShadesCard,
        type: 'primary',
      },
      // {
      //   ...this.wirelessAudioCard,
      //   type: 'danger',
      // },
      // {
      //   ...this.coffeeMakerCard,
      //   type: 'secondary',
      // },
    ],
  };

  sensor1: Observable<any>;
  sensor2: Observable<any>;
  sensor3: Observable<any>;

  actuador1: Observable<any>;
  actuador2: Observable<any>;
  actuador3: Observable<any>;
  actuador4: Observable<any>;

  actuadorValue;

  constructor(
    private themeService: NbThemeService,
    private dbService: FiredbService
  ) { }

  ngOnInit(){
    this.themeService.getJsTheme()
      .pipe(takeWhile(() => this.alive))
      .subscribe(theme => {
        this.statusCards = this.statusCardsByThemes[theme.name];
    });
    this.sensor1 = this.dbService.getSensor(this.nodoId, this.sensorId1);
    this.sensor2 = this.dbService.getSensor(this.nodoId, this.sensorId2);
    this.sensor3 = this.dbService.getSensor(this.nodoId, this.sensorId3);

    this.actuador1 = this.dbService.getActuador(this.nodoId, this.actuadorId1);
    this.actuador2 = this.dbService.getActuador(this.nodoId, this.actuadorId2);
    this.actuador3 = this.dbService.getActuador(this.nodoId, this.actuadorId3);
    this.actuador4 = this.dbService.getActuador(this.nodoId, this.actuadorId4);
    this.actuador1.subscribe(data =>{
      this.commonStatusCardsSet[0].status = data.Estado;
    });
    this.actuador2.subscribe(data =>{
      this.commonStatusCardsSet[1].status = data.Estado;
    });
    // this.actuador3.subscribe(data =>{
    //   this.commonStatusCardsSet[2].status = data.Estado;
    // });
    // this.actuador4.subscribe(data =>{
    //   this.commonStatusCardsSet[3].status = data.Estado;
    // });
  }

  ngOnDestroy() {
    this.alive = false;
  }

  toggle(statusCard){
    // alert(statusCard.title);
      statusCard.status = !statusCard.status;
      this.dbService.updateActuador(this.nodoId, statusCard.actuadorId, statusCard.status);
  }

  onCardClick(){
    alert('click');
  }

}
