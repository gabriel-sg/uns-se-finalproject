import { NgModule } from '@angular/core';
import { Routes, RouterModule } from '@angular/router';

import { NodosComponent } from './nodos.component';
import { Nodo1Component } from './nodo1/nodo1.component';
import { Nodo2Component } from './nodo2/nodo2.component';
import { Nodo3Component } from './nodo3/nodo3.component';
import { HomeComponent } from './home/home.component';


const routes: Routes = [{
  path: '',
  component: NodosComponent,
  children: [
    {
      path: 'nodo1',
      component: Nodo1Component,
    },
    {
      path: 'nodo2',
      component: Nodo2Component,
    },
    {
      path: 'nodo3',
      component: Nodo3Component,
    },
    {
      path: 'home',
      component: HomeComponent,
    }, {
      path: '',
      redirectTo: 'home',
      pathMatch: 'full',
    },
],
}];

@NgModule({
  imports: [RouterModule.forChild(routes)],
  exports: [RouterModule]
})
export class NodosRoutingModule { }

