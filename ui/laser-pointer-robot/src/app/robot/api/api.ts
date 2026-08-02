import { Component, inject } from '@angular/core';
import { CommandHandler } from '../../core/command-handler';

@Component({
  selector: 'app-api',
  imports: [],
  template: `
    <article>
      <h2>API Controller</h2>

      <section>
        <h3>startHoming</h3>
        <button (click)="service.startHoming()">Start Homing</button>
      </section>
      <section>
        <h3>abortMotion</h3>
        <button (click)="service.abortMotion()">Abort Motion</button>
      </section>
      <section>
        <h3>gotoPosition(x: number, y: number)</h3>
        <input type="number" #x placeholder="X" />
        <input type="number" #y placeholder="Y" />
        <button (click)="service.gotoPosition(x.valueAsNumber, y.valueAsNumber)">
          Go to Position
        </button>
      </section>

      <section>
        <h3>gotoPositionZero</h3>
        <button (click)="service.gotoPositionZero()">Go to Position Zero</button>
      </section>
      <section>
        <h3>setMicrosteps(value: number)</h3>
        <input type="number" #microsteps placeholder="Microsteps" />
        <button (click)="service.setMicrosteps(microsteps.valueAsNumber)">Set Microsteps</button>
      </section>
      <section>
        <h3>setSpeed(value: number)</h3>
        <input type="number" #speed placeholder="Speed" />
        <button (click)="service.setSpeed(speed.valueAsNumber)">Set Speed</button>
      </section>
      <section>
        <h3>setAcceleration(value: number)</h3>
        <input type="number" #acceleration placeholder="Acceleration" />
        <button (click)="service.setAcceleration(acceleration.valueAsNumber)">
          Set Acceleration
        </button>
      </section>

      <section>
        <h3>testPatternSquare</h3>
        <button (click)="service.testPatternSquare()">Test Pattern Square</button>
      </section>
      <section>
        <h3>testPatternDiamond</h3>
        <button (click)="service.testPatternDiamond()">Test Pattern Diamond</button>
      </section>
      <section>
        <h3>testPatternFigure8</h3>
        <button (click)="service.testPatternFigure8()">Test Pattern Figure 8</button>
      </section>
      <section>
        <h3>testPatternSpiral</h3>
        <button (click)="service.testPatternSpiral()">Test Pattern Spiral</button>
      </section>
      <section>
        <h3>printCatalog</h3>
        <button (click)="service.printCatalog()">Print Catalog</button>
      </section>
    </article>
  `,
  styles: ``,
})
export class ApiComponent {
  protected readonly service = inject(CommandHandler);
}
