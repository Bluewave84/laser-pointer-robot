import { Component, signal } from '@angular/core';
import { RouterOutlet } from '@angular/router';
import { ApiComponent } from './robot/api/api';

@Component({
  selector: 'app-root',
  imports: [RouterOutlet, ApiComponent],
  template: `
    <h1>Hello, {{ title() }}</h1>
    <app-api></app-api>
    <router-outlet />
  `,
  styles: [],
})
export class App {
  protected readonly title = signal('laser-pointer-robot');
}
